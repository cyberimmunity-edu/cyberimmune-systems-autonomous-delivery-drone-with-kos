#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/CredentialManagerInterface.idl.h>

nk_err_t GetRsaKeyImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_GetRsaKey_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_GetRsaKey_res *res, struct nk_arena *resArena);
nk_err_t SetRsaKeyImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_SetRsaKey_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_SetRsaKey_res *res, struct nk_arena *resArena);
nk_err_t SignMessageImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_SignMessage_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_SignMessage_res *res, struct nk_arena *resArena);
nk_err_t CheckSignatureImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_CheckSignature_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_CheckSignature_res *res, struct nk_arena *resArena);

static struct CredentialManagerInterface *CreateCredentialManagerInterfaceImpl(void) {
    static const struct CredentialManagerInterface_ops Ops = {
        .GetRsaKey = GetRsaKeyImpl, .SetRsaKey = SetRsaKeyImpl, .SignMessage = SignMessageImpl, .CheckSignature = CheckSignatureImpl
    };

    static CredentialManagerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}