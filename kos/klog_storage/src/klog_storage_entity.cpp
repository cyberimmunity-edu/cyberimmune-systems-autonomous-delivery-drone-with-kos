
#include <klog_storage/server.h>
#include <drone_controller/KlogStorageEntity.edl.h>

#include <kss/audit.h>

#include <nglog/nglog.h>

#include <stdlib.h>
#include <stdio.h>

#define RED     "\x1B[31m"
#define GRN     "\x1B[32m"
#define RESET   "\x1B[0m"


struct MyContext
{
    int some_important_data;
};

static int _write([[maybe_unused]] struct MyContext *ctx, const struct kl_KlogStorage_Entry *entry)
{
    switch (entry->reason)
    {
        case (KSS_AUDIT_POSITION_DENY):
            fprintf(stderr, RED "%s" RESET "\n", entry->msg);
            break;
        case (KSS_AUDIT_POSITION_GRANT):
            fprintf(stderr, GRN "%s" RESET "\n", entry->msg);
            break;
        default:
            fprintf(stderr, "%s\n", entry->msg);
    }

    return 0;
}

static int _read_range([[maybe_unused]] struct MyContext *ctx, [[maybe_unused]] nk_uint64_t first_id, [[maybe_unused]] nk_uint64_t last_id, [[maybe_unused]] struct kl_KlogStorage_Entry *entries)
{
    return 0;
}

static int _read([[maybe_unused]] struct MyContext *ctx, [[maybe_unused]] nk_uint32_t num_entries, [[maybe_unused]] struct kl_KlogStorage_Entry *entries)
{
    return 0;
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[])
{
    static struct MyContext ctx;
    nglog_init("KlogStorageEntity", NGLOG_FOREGROUND, NGLOG_VERBOSE);

    struct kl_KlogStorage *iface = klog_storage_IKlog_storage_dispatcher(&ctx,
                                                                         (kl_KlogStorage_write_func)_write,
                                                                         (kl_KlogStorage_read_func)_read,
                                                                         (kl_KlogStorage_read_range_func)_read_range);
    struct kl_KlogStorage_component *comp = klog_storage_storage_component(iface);

    return klog_storage_run(KLOG_STORAGE_SERVER_CONNECTION_ID, drone_controller_KlogStorageEntity_klogStorage_iidOffset, drone_controller_KlogStorageEntity_klogStorage_storage_iid, comp);
}
