#include "../include/credential_manager_interface.h"
#include "../include/credential_manager.h"

#include <string.h>

nk_err_t GetRsaKeyImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_GetRsaKey_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_GetRsaKey_res *res, struct nk_arena *resArena) {
    char e[CredentialManagerInterface_MaxKeyLength] = {0};
    char n[CredentialManagerInterface_MaxKeyLength] = {0};
    res->success = getRsaKey(e, n);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, resArena, &(res->e), strlen(e) + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(msg, e);

    msg = nk_arena_alloc(nk_char_t, resArena, &(res->n), strlen(n) + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(msg, n);

    return NK_EOK;
}

nk_err_t SetRsaKeyImpl(struct CredentialManagerInterface *self,
                        const CredentialManagerInterface_SetRsaKey_req *req, const struct nk_arena *reqArena,
                        CredentialManagerInterface_SetRsaKey_res *res, struct nk_arena *resArena) {
    char key[CredentialManagerInterface_MaxRawKeyLength] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->key), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(key, msg);

    res->success = setRsaKey(key);

    return NK_EOK;
}

nk_err_t SignMessageImpl(struct CredentialManagerInterface *self,
                        const CredentialManagerInterface_SignMessage_req *req, const struct nk_arena *reqArena,
                        CredentialManagerInterface_SignMessage_res *res, struct nk_arena *resArena) {
    char message[CredentialManagerInterface_MaxMessageLength] = {0};
    char signature[CredentialManagerInterface_MaxSignatureLength] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->message), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(message, msg);

    res->success = signMessage(message, signature);

    msg = nk_arena_alloc(nk_char_t, resArena, &(res->signature), strlen(signature) + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(msg, signature);

    return NK_EOK;
}

nk_err_t CheckSignatureImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_CheckSignature_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_CheckSignature_res *res, struct nk_arena *resArena) {
    char message[CredentialManagerInterface_MaxMessageLength] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->message), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(message, msg);

    res->success = checkSignature(message, res->correct);

    return NK_EOK;
}