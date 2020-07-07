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
#ifndef PXRUSDTRANSLATORS_MATERIAL_WRITER_H
#define PXRUSDTRANSLATORS_MATERIAL_WRITER_H

/// \file

#include <maya/MFnDependencyNode.h>

#include <pxr/pxr.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/timeCode.h>

#include <mayaUsd/fileio/shaderWriter.h>
#include <mayaUsd/fileio/writeJobContext.h>

PXR_NAMESPACE_OPEN_SCOPE

/// Shader writer for exporting Maya's material shading nodes to USD.
class PxrUsdTranslators_MaterialWriter : public UsdMayaShaderWriter
{
    public:
        PxrUsdTranslators_MaterialWriter(
                const MFnDependencyNode& depNodeFn,
                const SdfPath& usdPath,
                UsdMayaWriteJobContext& jobCtx);

        void Write(const UsdTimeCode& usdTime) override;

        TfToken GetShadingAttributeNameForMayaAttrName(
                const TfToken& mayaAttrName) override;
};


PXR_NAMESPACE_CLOSE_SCOPE


#endif