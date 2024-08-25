/**
 * \file
 * \~English
 * \brief Implementation of methods for work with a log.
 * \details The file contains implementation of methods, that are required to log messages.
 *
 * \~Russian
 * \brief Реализация методов для работы с логом.
 * \details В файле реализованы методы, необходимые для логирования сообщений.
 */

#include "../include/logger.h"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/spdlog.h>

/** \cond */
std::shared_ptr<spdlog::logger> logger;
/** \endcond */

int createLog() {
    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    consoleSink->set_formatter(std::make_unique<spdlog::pattern_formatter>(
        "[%Y-%m-%dT%H:%M:%S][%^%l%$]%v", spdlog::pattern_time_type::local, "\n"));
    consoleSink->set_level(spdlog::level::trace);
    auto fileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/logs/flight_controller.log", true);
    fileSink->set_formatter(std::make_unique<spdlog::pattern_formatter>(
        "[%Y-%m-%dT%H:%M:%S][%^%l%$]%v", spdlog::pattern_time_type::local, "\n"));
    fileSink->set_level(spdlog::level::trace);

    spdlog::sinks_init_list sinks {consoleSink, fileSink};
    logger = std::make_shared<spdlog::logger>("Flight Controller", sinks);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    spdlog::flush_on(spdlog::level::trace);

    return 1;
}

int addLogEntry(char* entry, int level) {
    switch (level) {
    case 0:
        SPDLOG_LOGGER_TRACE(logger, entry);
        break;
    case 1:
        SPDLOG_LOGGER_DEBUG(logger, entry);
        break;
    case 2:
        SPDLOG_LOGGER_INFO(logger, entry);
        break;
    case 3:
        SPDLOG_LOGGER_WARN(logger, entry);
        break;
    case 4:
        SPDLOG_LOGGER_ERROR(logger, entry);
        break;
    case 5:
        SPDLOG_LOGGER_CRITICAL(logger, entry);
        break;
    }

    return 1;
}