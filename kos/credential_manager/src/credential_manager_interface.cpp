/**
 * \file
 * \~English \brief Implementation of CredentialManagerInterface IDL-interface methods.
 * \~Russian \brief Реализация методов IDL-интерфейса CredentialManagerInterface.
 */

#include "../include/credential_manager_interface.h"
#include "../include/credential_manager.h"

#include <string.h>

nk_err_t SignMessageImpl(struct CredentialManagerInterface *self,
                        const CredentialManagerInterface_SignMessage_req *req, const struct nk_arena *reqArena,
                        CredentialManagerInterface_SignMessage_res *res, struct nk_arena *resArena) {
    char message[CredentialManagerInterface_MaxMessageLength + 1] = {0};
    char signature[CredentialManagerInterface_MaxSignatureLength + 1] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->message), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > CredentialManagerInterface_MaxMessageLength)
        return NK_ENOMEM;
    strncpy(message, msg, len);

    res->success = getMessageSignature(message, signature);

    len = strlen(signature);
    msg = nk_arena_alloc(nk_char_t, resArena, &(res->signature), len + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > CredentialManagerInterface_MaxSignatureLength)
        return NK_ENOMEM;
    strncpy(msg, signature, len);

    return NK_EOK;
}

nk_err_t CheckSignatureImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_CheckSignature_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_CheckSignature_res *res, struct nk_arena *resArena) {
    char message[CredentialManagerInterface_MaxMessageLength + 1] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->message), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > CredentialManagerInterface_MaxMessageLength)
        return NK_ENOMEM;
    strncpy(message, msg, len);

    res->success = checkMessageSignature(message, res->correct);

    return NK_EOK;
}