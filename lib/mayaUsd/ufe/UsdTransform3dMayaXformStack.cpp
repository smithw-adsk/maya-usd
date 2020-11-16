//
// Copyright 2020 Autodesk
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "UsdTransform3dMayaXformStack.h"

#include "private/UfeNotifGuard.h"

#include <mayaUsd/fileio/utils/xformStack.h>
#include <mayaUsd/ufe/Utils.h>

#include <maya/MEulerRotation.h>

#include <map>
#include <functional>

namespace {

using OpFunc = std::function<UsdGeomXformOp()>;

using namespace MayaUsd::ufe;

// Type traits for GfVec precision.
template <class V> struct OpPrecision
{
    static UsdGeomXformOp::Precision precision;
};

template <>
UsdGeomXformOp::Precision OpPrecision<GfVec3f>::precision = UsdGeomXformOp::PrecisionFloat;

template <>
UsdGeomXformOp::Precision OpPrecision<GfVec3d>::precision = UsdGeomXformOp::PrecisionDouble;

inline double TO_DEG(double a) { return a * 180.0 / 3.141592654; }
inline double TO_RAD(double a) { return a * 3.141592654 / 180.0; }

VtValue getValue(const UsdAttribute& attr, const UsdTimeCode& time)
{
    VtValue value;
    attr.Get(&value, time);
    return value;
}

// UsdMayaXformStack::FindOpIndex() requires an inconvenient isInvertedTwin
// argument, various rotate transform op equivalences in a separate
// UsdMayaXformStack::IsCompatibleType().  Just roll our own op name to
// Maya transform stack index position.
const std::unordered_map<std::string, UsdTransform3dMayaXformStack::OpNdx> opNameToNdx {
    { "xformOp:translate", UsdTransform3dMayaXformStack::NdxTranslate },
    { "xformOp:translate:rotatePivotTranslate",
      UsdTransform3dMayaXformStack::NdxRotatePivotTranslate },
    { "xformOp:translate:rotatePivot", UsdTransform3dMayaXformStack::NdxRotatePivot },
    { "xformOp:rotateX", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateY", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateZ", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateXYZ", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateXZY", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateYXZ", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateYZX", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateZXY", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateZYX", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:orient", UsdTransform3dMayaXformStack::NdxRotate },
    { "xformOp:rotateXYZ:rotateAxis", UsdTransform3dMayaXformStack::NdxRotateAxis },
    { "!invert!xformOp:translate:rotatePivot",
      UsdTransform3dMayaXformStack::NdxRotatePivotInverse },
    { "xformOp:translate:scalePivotTranslate",
      UsdTransform3dMayaXformStack::NdxScalePivotTranslate },
    { "xformOp:translate:scalePivot", UsdTransform3dMayaXformStack::NdxScalePivot },
    { "xformOp:shear", UsdTransform3dMayaXformStack::NdxShear },
    { "xformOp:scale", UsdTransform3dMayaXformStack::NdxScale },
    { "!invert!xformOp:translate:scalePivot", UsdTransform3dMayaXformStack::NdxScalePivotInverse }
};

//----------------------------------------------------------------------
// Conversion functions from RotXYZ to all supported rotation attributes.
//----------------------------------------------------------------------

typedef VtValue (*CvtRotXYZToAttrFn)(double x, double y, double z);

VtValue toXYZ(double x, double y, double z)
{
    // No rotation order conversion
    VtValue v;
    v = GfVec3f(x, y, z);
    return v;
}

// Reorder argument RotXYZ rotation.
template <MEulerRotation::RotationOrder DST_ROT_ORDER> VtValue to(double x, double y, double z)
{
    MEulerRotation eulerRot(TO_RAD(x), TO_RAD(y), TO_RAD(z), MEulerRotation::kXYZ);
    eulerRot.reorderIt(DST_ROT_ORDER);
    VtValue v;
    v = GfVec3f(TO_DEG(eulerRot.x), TO_DEG(eulerRot.y), TO_DEG(eulerRot.z));
    return v;
}

auto toXZY = to<MEulerRotation::kXZY>;
auto toYXZ = to<MEulerRotation::kYXZ>;
auto toYZX = to<MEulerRotation::kYZX>;
auto toZXY = to<MEulerRotation::kZXY>;
auto toZYX = to<MEulerRotation::kZYX>;

// Scalar float is the proper type for single-axis rotations.
VtValue toX(double x, double, double)
{
    VtValue v;
    v = float(x);
    return v;
}

VtValue toY(double, double y, double)
{
    VtValue v;
    v = float(y);
    return v;
}

VtValue toZ(double, double, double z)
{
    VtValue v;
    v = float(z);
    return v;
}

CvtRotXYZToAttrFn getCvtRotXYZToAttrFn(const TfToken& opName)
{
    // Can't get std::unordered_map<TfToken, CvtRotXYZToAttrFn> to instantiate.
    static std::map<TfToken, CvtRotXYZToAttrFn> cvt = {
        { TfToken("xformOp:rotateX"), toX },     { TfToken("xformOp:rotateY"), toY },
        { TfToken("xformOp:rotateZ"), toZ },     { TfToken("xformOp:rotateXYZ"), toXYZ },
        { TfToken("xformOp:rotateXZY"), toXZY }, { TfToken("xformOp:rotateYXZ"), toYXZ },
        { TfToken("xformOp:rotateYZX"), toYZX }, { TfToken("xformOp:rotateZXY"), toZXY },
        { TfToken("xformOp:rotateZYX"), toZYX }, { TfToken("xformOp:orient"), nullptr }
    }; // FIXME, unsupported.

    return cvt.at(opName);
}

//----------------------------------------------------------------------
// Conversion functions from all supported rotation attributes to RotXYZ.
//----------------------------------------------------------------------

typedef Ufe::Vector3d (*CvtRotXYZFromAttrFn)(const VtValue& value);

Ufe::Vector3d fromXYZ(const VtValue& value)
{
    // No rotation order conversion
    auto v = value.Get<GfVec3f>();
    return Ufe::Vector3d(v[0], v[1], v[2]);
}

template <MEulerRotation::RotationOrder SRC_ROT_ORDER> Ufe::Vector3d from(const VtValue& value)
{
    auto v = value.Get<GfVec3f>();

    MEulerRotation eulerRot(TO_RAD(v[0]), TO_RAD(v[1]), TO_RAD(v[2]), SRC_ROT_ORDER);
    eulerRot.reorderIt(MEulerRotation::kXYZ);
    return Ufe::Vector3d(TO_DEG(eulerRot.x), TO_DEG(eulerRot.y), TO_DEG(eulerRot.z));
}

auto fromXZY = from<MEulerRotation::kXZY>;
auto fromYXZ = from<MEulerRotation::kYXZ>;
auto fromYZX = from<MEulerRotation::kYZX>;
auto fromZXY = from<MEulerRotation::kZXY>;
auto fromZYX = from<MEulerRotation::kZYX>;

Ufe::Vector3d fromX(const VtValue& value) { return Ufe::Vector3d(value.Get<float>(), 0, 0); }

Ufe::Vector3d fromY(const VtValue& value) { return Ufe::Vector3d(0, value.Get<float>(), 0); }

Ufe::Vector3d fromZ(const VtValue& value) { return Ufe::Vector3d(0, 0, value.Get<float>()); }

CvtRotXYZFromAttrFn getCvtRotXYZFromAttrFn(const TfToken& opName)
{
    static std::map<TfToken, CvtRotXYZFromAttrFn> cvt = {
        { TfToken("xformOp:rotateX"), fromX },     { TfToken("xformOp:rotateY"), fromY },
        { TfToken("xformOp:rotateZ"), fromZ },     { TfToken("xformOp:rotateXYZ"), fromXYZ },
        { TfToken("xformOp:rotateXZY"), fromXZY }, { TfToken("xformOp:rotateYXZ"), fromYXZ },
        { TfToken("xformOp:rotateYZX"), fromYZX }, { TfToken("xformOp:rotateZXY"), fromZXY },
        { TfToken("xformOp:rotateZYX"), fromZYX }, { TfToken("xformOp:orient"), nullptr }
    }; // FIXME, unsupported.

    return cvt.at(opName);
}

} // namespace

namespace MAYAUSD_NS_DEF {
namespace ufe {

namespace {

inline Ufe::Transform3d::Ptr
nextTransform3d(const Ufe::Transform3dHandler::Ptr& nextHandler, const Ufe::SceneItem::Ptr& item)
{
    return nextHandler->transform3d(item);
}

inline Ufe::Transform3d::Ptr nextEditTransform3d(
    const Ufe::Transform3dHandler::Ptr& nextHandler,
    const Ufe::SceneItem::Ptr&          item)
{
    return nextHandler->editTransform3d(item);
}

typedef Ufe::Transform3d::Ptr (*NextTransform3dFn)(
    const Ufe::Transform3dHandler::Ptr& nextHandler,
    const Ufe::SceneItem::Ptr&          item);

Ufe::Transform3d::Ptr createTransform3d(
    const Ufe::Transform3dHandler::Ptr& nextHandler,
    const Ufe::SceneItem::Ptr&          item,
    NextTransform3dFn                   nextTransform3dFn)
{
    UsdSceneItem::Ptr usdItem = std::dynamic_pointer_cast<UsdSceneItem>(item);
#if !defined(NDEBUG)
    assert(usdItem);
#endif

    // If the prim isn't transformable, can't create a Transform3d interface
    // for it.
    UsdGeomXformable xformSchema(usdItem->prim());
    if (!xformSchema) {
        return nullptr;
    }
    bool resetsXformStack = false;
    auto xformOps = xformSchema.GetOrderedXformOps(&resetsXformStack);

    // Early out: if there are no transform ops yet, it's a match.
    if (xformOps.empty()) {
        return UsdTransform3dMayaXformStack::create(usdItem);
    }

    // If the prim supports the Maya transform stack, create a Maya transform
    // stack interface for it, otherwise delegate to the next handler in the
    // chain of responsibility.
    auto stackOps = UsdMayaXformStack::MayaStack().MatchingSubstack(xformOps);

    return stackOps.empty() ? nextTransform3dFn(nextHandler, item)
                            : UsdTransform3dMayaXformStack::create(usdItem);
}

// Helper class to factor out common code for translate, rotate, scale
// undoable commands.
class UsdTRSUndoableCmdBase : public Ufe::SetVector3dUndoableCommand
{
private:
    const UsdTimeCode fReadTime;
    const UsdTimeCode fWriteTime;
    const TfToken     fAttrName;

public:

    UsdGeomXformOp    _op;
    VtValue           _prevOpValue;
    VtValue           _newOpValue;

	struct State {
		virtual const char* name() const = 0;
		virtual void handleUndo(UsdTRSUndoableCmdBase*) {
			TF_CODING_ERROR("handleUndo() called for illegal state '%s' in UsdTRSUndoableCmdBase", name());
		} 
		virtual void handleSet(UsdTRSUndoableCmdBase*, const VtValue&) {
			TF_CODING_ERROR("handleSet() called for illegal state '%s' in UsdTRSUndoableCmdBase", name()); }
	};

	struct InitialState : public State {
		const char* name() const override { return "initial"; }
		void handleUndo(UsdTRSUndoableCmdBase* cmd) override {
			// Maya triggers an undo on command creation, ignore it.
			cmd->_state = &UsdTRSUndoableCmdBase::_initialUndoCalledState;
		}
		void handleSet(UsdTRSUndoableCmdBase* cmd, const VtValue& v) override {
			// Going from initial to executing / executed state, save value.
			cmd->_prevOpValue = getValue(cmd->_op.GetAttr(), cmd->readTime());
			cmd->_newOpValue  = v;
			cmd->setValue(v);
			cmd->_state = &UsdTRSUndoableCmdBase::_executeState;
		}
	};

	struct InitialUndoCalledState : public State {
		const char* name() const override { return "initial undo called"; }
		void handleSet(UsdTRSUndoableCmdBase* cmd, const VtValue&) override {
			// Maya triggers a redo on command creation, ignore it.
			cmd->_state = &UsdTRSUndoableCmdBase::_initialState;
		}
	};

	struct ExecuteState : public State {
		const char* name() const override { return "execute"; }
		void handleUndo(UsdTRSUndoableCmdBase* cmd) override {
			cmd->recreateOp();
			cmd->setValue(cmd->_prevOpValue);
			cmd->_state = &UsdTRSUndoableCmdBase::_undoneState;
		}
		void handleSet(UsdTRSUndoableCmdBase* cmd, const VtValue& v) override {
			cmd->_newOpValue = v;
			cmd->setValue(v);
		}
	};

	struct UndoneState : public State {
		const char* name() const override { return "undone"; }
		void handleSet(UsdTRSUndoableCmdBase* cmd, const VtValue&) override {
			// Can ignore the value, we already have it --- or assert they're
			// equal, perhaps.
			cmd->recreateOp();
			cmd->setValue(cmd->_newOpValue);
			cmd->_state = &UsdTRSUndoableCmdBase::_redoneState;
		}
	};

	struct RedoneState : public State {
		const char* name() const override { return "redone"; }
		void handleUndo(UsdTRSUndoableCmdBase* cmd) override {
			cmd->recreateOp();
			cmd->setValue(cmd->_prevOpValue);
			cmd->_state = &UsdTRSUndoableCmdBase::_undoneState;
		}
	};

    UsdTRSUndoableCmdBase(
        const VtValue&        newOpValue,
        const Ufe::Path&      path,
        const UsdGeomXformOp& op,
        const UsdTimeCode&    writeTime_)
        : Ufe::SetVector3dUndoableCommand(path)
        // Always read from proxy shape time.
        , fReadTime(getTime(path))
        , fWriteTime(writeTime_)
        , fAttrName(op.GetOpName())
        , _op(op)
        , _newOpValue(newOpValue)
    {}

    void recreateOp()
    {
        auto sceneItem
            = std::dynamic_pointer_cast<UsdSceneItem>(Ufe::Hierarchy::createItem(path()));
        TF_AXIOM(sceneItem);
        auto prim = sceneItem->prim();
        TF_AXIOM(prim);
        auto attr = prim.GetAttribute(fAttrName);
        TF_AXIOM(attr);
        _op = UsdGeomXformOp(attr);
    }

    void undo() override { _state->handleUndo(this); }

    void redo() override
    {
		TF_CODING_ERROR("Illegal call to redo() in UsdTRSUndoableCmdBase.");
    }

	void handleSet(const VtValue& v) { _state->handleSet(this, v); }

    void setValue(const VtValue& v) { _op.GetAttr().Set(v, fWriteTime); }

    UsdTimeCode readTime() const { return fReadTime; }
    UsdTimeCode writeTime() const { return fWriteTime; }

	static InitialState           _initialState;
	static InitialUndoCalledState _initialUndoCalledState;
	static ExecuteState           _executeState;
	static UndoneState            _undoneState;
	static RedoneState            _redoneState;

	State* _state{&_initialState};
};

UsdTRSUndoableCmdBase::InitialState UsdTRSUndoableCmdBase::_initialState;
UsdTRSUndoableCmdBase::InitialUndoCalledState UsdTRSUndoableCmdBase::_initialUndoCalledState;
UsdTRSUndoableCmdBase::ExecuteState UsdTRSUndoableCmdBase::_executeState;
UsdTRSUndoableCmdBase::UndoneState  UsdTRSUndoableCmdBase::_undoneState;
UsdTRSUndoableCmdBase::RedoneState  UsdTRSUndoableCmdBase::_redoneState;

// UsdRotatePivotTranslateUndoableCmd uses hard-coded USD common transform API
// single pivot attribute name, not reusable.
template <class V> class UsdVecOpUndoableCmd : public UsdTRSUndoableCmdBase
{
public:
    UsdVecOpUndoableCmd(
        const V&           v,
        const Ufe::Path&   path,
        OpFunc             opFunc,
        const UsdTimeCode& writeTime)
        : UsdTRSUndoableCmdBase(VtValue(v), path, opFunc(), writeTime)
    {}

    // Executes the command by setting the translation onto the transform op.
    bool set(double x, double y, double z) override
    {
        VtValue v;
        v = V(x, y, z);
        handleSet(v);
        return true;
    }
};

class UsdRotateOpUndoableCmd : public UsdTRSUndoableCmdBase
{
public:
    UsdRotateOpUndoableCmd(
        const GfVec3f&     r,
        const Ufe::Path&   path,
        OpFunc             opFunc,
        CvtRotXYZToAttrFn  cvt,
        const UsdTimeCode& writeTime)
      : UsdTRSUndoableCmdBase(VtValue(r), path, opFunc(), writeTime)
      , _cvtRotXYZToAttr(cvt)
    {}

    // Executes the command by setting the rotation onto the transform op.
    bool set(double x, double y, double z) override
    {
        VtValue v;
        v = _cvtRotXYZToAttr(x, y, z);
        handleSet(v);
        return true;
    }

private:
    // Convert from UFE RotXYZ rotation to a value for the transform op.
    CvtRotXYZToAttrFn _cvtRotXYZToAttr;
};

} // namespace

UsdTransform3dMayaXformStack::UsdTransform3dMayaXformStack(const UsdSceneItem::Ptr& item)
    : UsdTransform3dBase(item)
    , _xformable(prim())
{
    TF_AXIOM(_xformable);

    // *** FIXME ***  Consider receiving the ordered transform ops as a ctor
    // argument, as we're already asking for them.

    // *** FIXME ***  We ask for ordered transform ops, but the prim may have
    // other transform ops that are not in the ordered list.  However, those
    // transform ops are not contributing to the final local transform.
    bool resetsXformStack = false;
    auto xformOps = _xformable.GetOrderedXformOps(&resetsXformStack);
    for (const auto& op : xformOps) {
        std::string opName = op.GetOpName();
        auto        ndx = opNameToNdx.at(opName);
        _orderedOps[ndx] = op;
    }
}

/* static */
UsdTransform3dMayaXformStack::Ptr
UsdTransform3dMayaXformStack::create(const UsdSceneItem::Ptr& item)
{
    return std::make_shared<UsdTransform3dMayaXformStack>(item);
}

Ufe::Vector3d UsdTransform3dMayaXformStack::translation() const
{
    return getVector3d<GfVec3d>(UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTranslate));
}

Ufe::Vector3d UsdTransform3dMayaXformStack::rotation() const
{
    if (!hasOp(NdxRotate)) {
        return Ufe::Vector3d(0, 0, 0);
    }
    UsdGeomXformOp r = getOp(NdxRotate);
    TF_AXIOM(r);

    CvtRotXYZFromAttrFn cvt = getCvtRotXYZFromAttrFn(r.GetOpName());
    return cvt(getValue(r.GetAttr(), getTime(path())));
}

Ufe::Vector3d UsdTransform3dMayaXformStack::scale() const
{
    if (!hasOp(NdxScale)) {
        return Ufe::Vector3d(1, 1, 1);
    }
    UsdGeomXformOp s = getOp(NdxScale);
    TF_AXIOM(s);

    GfVec3f v;
    s.Get(&v, getTime(path()));
    return toUfe(v);
}

Ufe::TranslateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::translateCmd(double x, double y, double z)
{
    return setVector3dCmd(
        GfVec3d(x, y, z), UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTranslate));
}

Ufe::RotateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::rotateCmd(double x, double y, double z)
{
    GfVec3f v(x, y, z);
    bool hasRotate = hasOp(NdxRotate);
    // If there is no rotate transform op, we will create a RotXYZ.
    CvtRotXYZToAttrFn cvt = hasRotate ?
        getCvtRotXYZToAttrFn(getOp(NdxRotate).GetOpName()) : toXYZ;
    OpFunc f = hasRotate ? OpFunc([this](){ return getOp(NdxRotate); }) :
        OpFunc([this, v](){
        // Use notification guard, otherwise will generate one notification
        // for the xform op add, and another for the reorder.
        InTransform3dChange guard(path());
        auto r = _xformable.AddRotateXYZOp(); TF_AXIOM(r);
        r.Set(v);
        setXformOpOrder();
        return r; });

    return std::make_shared<UsdRotateOpUndoableCmd>(v, path(), std::move(f), cvt, UsdTimeCode::Default());
}

Ufe::ScaleUndoableCommand::Ptr UsdTransform3dMayaXformStack::scaleCmd(double x, double y, double z)
{
    GfVec3f v(x, y, z);
    OpFunc f = hasOp(NdxScale) ? OpFunc([this](){ return getOp(NdxScale); }) :
    OpFunc([this, v]() {
        InTransform3dChange guard(path());
        auto s = _xformable.AddScaleOp(); TF_AXIOM(s);
        s.Set(v);
        setXformOpOrder();
        return s; });

    return std::make_shared<UsdVecOpUndoableCmd<GfVec3f>>(v, path(), std::move(f), UsdTimeCode::Default());
}

Ufe::TranslateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::rotatePivotCmd(double x, double y, double z)
{
    return pivotCmd(UsdMayaXformStackTokens->rotatePivot, x, y, z);
}

Ufe::Vector3d UsdTransform3dMayaXformStack::rotatePivot() const
{
    return getVector3d<GfVec3f>(UsdGeomXformOp::GetOpName(
        UsdGeomXformOp::TypeTranslate, UsdMayaXformStackTokens->rotatePivot));
}

Ufe::TranslateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::scalePivotCmd(double x, double y, double z)
{
    return pivotCmd(UsdMayaXformStackTokens->scalePivot, x, y, z);
}

Ufe::Vector3d UsdTransform3dMayaXformStack::scalePivot() const
{
    return getVector3d<GfVec3f>(UsdGeomXformOp::GetOpName(
        UsdGeomXformOp::TypeTranslate, UsdMayaXformStackTokens->scalePivot));
}

Ufe::TranslateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::translateRotatePivotCmd(double x, double y, double z)
{
    auto opSuffix = UsdMayaXformStackTokens->rotatePivotTranslate;
    auto attrName = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTranslate, opSuffix);
    return setVector3dCmd(GfVec3f(x, y, z), attrName, opSuffix);
}

Ufe::Vector3d UsdTransform3dMayaXformStack::rotatePivotTranslation() const
{
    return getVector3d<GfVec3f>(UsdGeomXformOp::GetOpName(
        UsdGeomXformOp::TypeTranslate, UsdMayaXformStackTokens->rotatePivotTranslate));
}

Ufe::TranslateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::translateScalePivotCmd(double x, double y, double z)
{
    auto opSuffix = UsdMayaXformStackTokens->scalePivotTranslate;
    auto attrName = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTranslate, opSuffix);
    return setVector3dCmd(GfVec3f(x, y, z), attrName, opSuffix);
}

Ufe::Vector3d UsdTransform3dMayaXformStack::scalePivotTranslation() const
{
    return getVector3d<GfVec3f>(UsdGeomXformOp::GetOpName(
        UsdGeomXformOp::TypeTranslate, UsdMayaXformStackTokens->scalePivotTranslate));
}

template <class V>
Ufe::Vector3d UsdTransform3dMayaXformStack::getVector3d(const TfToken& attrName) const
{
    // If the attribute doesn't exist yet, return a zero vector.
    auto attr = prim().GetAttribute(attrName);
    if (!attr) {
        return Ufe::Vector3d(0, 0, 0);
    }

    UsdGeomXformOp op(attr);
    TF_AXIOM(op);

    V v;
    op.Get(&v, getTime(path()));
    return toUfe(v);
}

template <class V>
Ufe::SetVector3dUndoableCommand::Ptr UsdTransform3dMayaXformStack::setVector3dCmd(
    const V&       v,
    const TfToken& attrName,
    const TfToken& opSuffix)
{
    auto attr = prim().GetAttribute(attrName);
    OpFunc f = attr ? OpFunc([this, attr](){ return UsdGeomXformOp(attr); }) :
    OpFunc([this, opSuffix, v](){
        InTransform3dChange guard(path());
        auto op = _xformable.AddTranslateOp(
                OpPrecision<V>::precision, opSuffix); TF_AXIOM(op);
        op.Set(v);
        setXformOpOrder();
        return op; });

    return std::make_shared<UsdVecOpUndoableCmd<V>>(v, path(), std::move(f), UsdTimeCode::Default());
}

Ufe::TranslateUndoableCommand::Ptr
UsdTransform3dMayaXformStack::pivotCmd(const TfToken& pvtOpSuffix, double x, double y, double z)
{
    auto pvtAttrName = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTranslate, pvtOpSuffix);

    GfVec3f v(x, y, z);
    auto attr = prim().GetAttribute(pvtAttrName);
    OpFunc f = attr ? OpFunc([this, attr](){ return UsdGeomXformOp(attr); }) :
        OpFunc([this, pvtOpSuffix, v](){
            // Without a notification guard each operation (each transform op
            // addition, setting the attribute value, and setting the transform
            // op order) will notify.  Observers would see an object in an
            // inconsistent state, especially after pivot is added but before
            // its inverse is added --- this does not match the Maya transform
            // stack.  Use of SdfChangeBlock is discouraged when calling USD
            // APIs above Sdf, so use our own guard.
            InTransform3dChange guard(path());
            auto p = _xformable.AddTranslateOp(
                UsdGeomXformOp::PrecisionFloat, pvtOpSuffix);
            auto pInv = _xformable.AddTranslateOp(
                UsdGeomXformOp::PrecisionFloat, pvtOpSuffix, /* isInverseOp */ true);
            TF_AXIOM(p && pInv);
            p.Set(v);
            setXformOpOrder();
            return p; });

    return std::make_shared<UsdVecOpUndoableCmd<GfVec3f>>(v, path(), std::move(f), UsdTimeCode::Default());
}

void UsdTransform3dMayaXformStack::setXformOpOrder()
{
    // Simply adding a transform op appends to the op order vector.  Therefore,
    // after addition, we must sort the ops to preserve Maya transform stack
    // ordering.  Use the Maya transform stack indices to add to a map, then
    // simply traverse the map to obtain the transform ops in order.
    std::map<int, UsdGeomXformOp> orderedOps;
    bool                          resetsXformStack = false;
    auto                          oldOrder = _xformable.GetOrderedXformOps(&resetsXformStack);
    for (const auto& op : oldOrder) {
        std::string opName = op.GetOpName();
        auto        ndx = opNameToNdx.at(opName);
        orderedOps[ndx] = op;
    }

    // Set the transform op order attribute, and rebuild our indexed cache.
    _orderedOps.clear();
    std::vector<UsdGeomXformOp> newOrder;
    newOrder.reserve(oldOrder.size());
    for (const auto& orderedOp : orderedOps) {
        const auto& op = orderedOp.second;
        newOrder.emplace_back(op);
        std::string opName = op.GetOpName();
        auto        ndx = opNameToNdx.at(opName);
        _orderedOps[ndx] = op;
    }

    _xformable.SetXformOpOrder(newOrder, resetsXformStack);
}

bool UsdTransform3dMayaXformStack::hasOp(OpNdx ndx) const
{
    return _orderedOps.find(ndx) != _orderedOps.end();
}

UsdGeomXformOp UsdTransform3dMayaXformStack::getOp(OpNdx ndx) const { return _orderedOps.at(ndx); }

//------------------------------------------------------------------------------
// UsdTransform3dMayaXformStackHandler
//------------------------------------------------------------------------------

UsdTransform3dMayaXformStackHandler::UsdTransform3dMayaXformStackHandler(
    const Ufe::Transform3dHandler::Ptr& nextHandler)
    : Ufe::Transform3dHandler()
    , _nextHandler(nextHandler)
{
}

/*static*/
UsdTransform3dMayaXformStackHandler::Ptr
UsdTransform3dMayaXformStackHandler::create(const Ufe::Transform3dHandler::Ptr& nextHandler)
{
    return std::make_shared<UsdTransform3dMayaXformStackHandler>(nextHandler);
}

Ufe::Transform3d::Ptr
UsdTransform3dMayaXformStackHandler::transform3d(const Ufe::SceneItem::Ptr& item) const
{
    return createTransform3d(_nextHandler, item, nextTransform3d);
}

Ufe::Transform3d::Ptr
UsdTransform3dMayaXformStackHandler::editTransform3d(const Ufe::SceneItem::Ptr& item) const
{
    return createTransform3d(_nextHandler, item, nextEditTransform3d);
}

} // namespace ufe
} // namespace MAYAUSD_NS_DEF
