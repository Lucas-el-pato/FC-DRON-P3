/**
 ******************************************************************************
 * @file    bal_link.c
 * @brief   Parser de comandos del balancin (ver bal_link.h).
 ******************************************************************************
 */

#include "bal_link.h"
#include "console.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#define BAL_LINK_LINE_MAX   32u
#define BAL_LINK_REPLY_MAX  256u

static char     s_line[BAL_LINK_LINE_MAX];
static uint32_t s_len = 0u;
static bool     s_overflow = false;

static char to_upper(char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char)(c - 'a' + 'A') : c;
}

static const char *skip_spaces(const char *p)
{
    while ((*p == ' ') || (*p == '\t')) {
        ++p;
    }
    return p;
}

/* "[+-]digitos[.digitos]" seguido solo de espacios. nano.specs no trae
 * scanf con float, por eso a mano.                                        */
static bool parse_number(const char *p, float *out)
{
    p = skip_spaces(p);

    float sign = 1.0f;
    if ((*p == '+') || (*p == '-')) {
        sign = (*p == '-') ? -1.0f : 1.0f;
        ++p;
    }

    bool digits = false;
    float v = 0.0f;
    while ((*p >= '0') && (*p <= '9')) {
        v = (v * 10.0f) + (float)(*p - '0');
        digits = true;
        ++p;
    }
    if ((*p == '.') || (*p == ',')) {
        ++p;
        float scale = 0.1f;
        while ((*p >= '0') && (*p <= '9')) {
            v += (float)(*p - '0') * scale;
            scale *= 0.1f;
            digits = true;
            ++p;
        }
    }

    if (!digits || (*skip_spaces(p) != '\0')) {
        return false;
    }
    *out = sign * v;
    return true;
}

static bool word_is(const char *p, const char *word)
{
    while (*word != '\0') {
        if (to_upper(*p) != *word) {
            return false;
        }
        ++p;
        ++word;
    }
    return *skip_spaces(p) == '\0';
}

static void parse_line(const char *line, bal_cmd_t *cmd)
{
    cmd->type = BAL_CMD_INVALID;
    cmd->value = 0.0f;

    const char *p = skip_spaces(line);

    if (word_is(p, "A") || word_is(p, "ARM")) {
        cmd->type = BAL_CMD_ARM;
    } else if (word_is(p, "D") || word_is(p, "STOP")) {
        cmd->type = BAL_CMD_STOP;
    } else if (word_is(p, "?")) {
        cmd->type = BAL_CMD_STATUS;
    } else if (word_is(p, "Z")) {
        cmd->type = BAL_CMD_ZERO;
    } else if (to_upper(*p) == 'R') {
        if (parse_number(p + 1, &cmd->value)) {
            cmd->type = BAL_CMD_REF;
        }
    } else if (to_upper(*p) == 'T') {
        if (parse_number(p + 1, &cmd->value)) {
            cmd->type = BAL_CMD_THROTTLE;
        }
    }
}

void bal_link_init(void)
{
    s_len = 0u;
    s_overflow = false;
}

bool bal_link_poll(bal_cmd_t *cmd)
{
    if (cmd == 0) {
        return false;
    }

    uint8_t c;
    while (console_read_byte(&c)) {
        if ((c == '\r') || (c == '\n')) {
            if ((s_len == 0u) && !s_overflow) {
                continue;   /* linea vacia o el \n de un \r\n */
            }
            s_line[s_len] = '\0';
            if (s_overflow) {
                cmd->type = BAL_CMD_INVALID;
                cmd->value = 0.0f;
            } else {
                parse_line(s_line, cmd);
            }
            s_len = 0u;
            s_overflow = false;
            return true;
        }

        if (s_len < (BAL_LINK_LINE_MAX - 1u)) {
            s_line[s_len++] = (char)c;
        } else {
            s_overflow = true;
        }
    }

    return false;
}

void bal_link_reply(const char *fmt, ...)
{
    char buf[BAL_LINK_REPLY_MAX];

    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        (void)console_try_print(buf);
    }
}
