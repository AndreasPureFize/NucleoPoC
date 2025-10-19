#include "DataLink_CLI.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>
#include "main.h"   /* HAL_GetTick, HAL_Delay */
#include "usart.h"  /* extern UART_HandleTypeDef huart2 (console) */
#include "DataLink_User.h" /* mid-level functions: SetPower, ReadOutputState, etc. */

/* ================================
 * UART console configuration
 * ================================ */
#ifndef CLI_UART_TX_TIMEOUT_MS
#define CLI_UART_TX_TIMEOUT_MS 100U
#endif
#ifndef CLI_UART_RX_TIMEOUT_MS
#define CLI_UART_RX_TIMEOUT_MS HAL_MAX_DELAY
#endif
#ifndef CLI_MAX_LINE
#define CLI_MAX_LINE 100U
#endif

/* ================================
 * Utilities
 * ================================ */
void ToUpperCase(char *str)
{
    if (str == NULL)
    {
        return;
    }
    while (*str != '\0')
    {
        if ((*str >= 'a') && (*str <= 'z'))
        {
            *str = (char)(*str - ('a' - 'A'));
        }
        str++;
    }
}

static void trim_inplace(char *s)
{
    size_t n;
    char *p;

    if (s == NULL)
    {
        return;
    }

    p = s;
    while ((*p != '\0') && ((*p == ' ')
                          || (*p == '\t')
                          || (*p == '\r')
                          || (*p == '\n')))
    {
        p++;
    }
    if (p != s)
    {
        (void)memmove(s, p, strlen(p) + 1U);
    }

    n = strlen(s);
    while ((n > 0U) && ((s[n - 1U] == ' ')
                     || (s[n - 1U] == '\t')
                     || (s[n - 1U] == '\r')
                     || (s[n - 1U] == '\n')))
    {
        s[n - 1U] = '\0';
        n--;
    }
}

/* Tokenize into up to 4 tokens separated by spaces/tabs. Returns count. */
static int split_tokens(char *line, char *tok[], int max_tok)
{
    int count = 0;
    char *p = line;

    while ((*p != '\0') && (count < max_tok))
    {
        while ((*p == ' ') || (*p == '\t'))
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }

        tok[count] = p;
        count++;

        while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        *p = '\0';
        p++;
    }

    return count;
}

/* ================================
 * HELP text
 * ================================ */
void CLI_PrintHelp(void)
{
    static const char help[] =
        "Commands:\r\n"
        " CONFIG CHANNEL <1 - 4>\r\n"
        " CONFIG POWER <0.5 - 1.0>\r\n"
        " READ CONFIG\r\n"
        " READ DATA\r\n"
        " READ ERRORS\r\n"
        " RESET ERRORS\r\n"
        " SET OUTPUT <0|1>\r\n"
        " READ OUTPUT\r\n"
        " SET DEFAULT <0|1>\r\n"
        " READ DEFAULT\r\n"
        " HELP\r\n"
        " X | EXIT\r\n";

    (void)HAL_UART_Transmit(&huart2, (uint8_t*)help, (uint16_t)(sizeof(help) - 1U), CLI_UART_TX_TIMEOUT_MS);
}

/* ================================
 * Prompt & Line input
 * ================================ */
/* Reads a line into buf (cap bytes). Handles echo, backspace, CR/LF.
   Returns true if a line was successfully read (non-empty or empty line). */
bool CLI_ReadLine(char *buf, uint16_t cap)
{
    uint16_t idx = 0U;

    if ((buf == NULL) || (cap == 0U))
    {
        return false;
    }

    (void)memset(buf, 0, cap);

    while(1)
    {
        uint8_t ch = 0U;

        if (HAL_UART_Receive(&huart2, &ch, 1U, CLI_UART_RX_TIMEOUT_MS) != HAL_OK)
        {
            return false;
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            static const char crlf[] = "\r\n";
            (void)HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2U, CLI_UART_TX_TIMEOUT_MS);
            buf[idx] = '\0';
            return true;
        }

        if ((ch == 0x08U) || (ch == 0x7FU))
        {
            if (idx > 0U)
            {
                static const char bs_erase[] = "\b \b";
                idx--;
                buf[idx] = '\0';
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)bs_erase, (uint16_t)(sizeof(bs_erase) - 1U), CLI_UART_TX_TIMEOUT_MS);
            }
            continue;
        }

        if (idx < (cap - 1U))
        {
            buf[idx] = (char)ch;
            idx++;
            (void)HAL_UART_Transmit(&huart2, &ch, 1U, CLI_UART_TX_TIMEOUT_MS);
        }
        else
        {
            static const char trunc_msg[] = "\r\n(line truncated)\r\n";
            (void)HAL_UART_Transmit(&huart2, (uint8_t*)trunc_msg, (uint16_t)(sizeof(trunc_msg) - 1U), CLI_UART_TX_TIMEOUT_MS);
        }
    }
}

/* ================================
 * Parser
 * ================================ */
static bool parse_int(const char *s, int *out)
{
    char *end = NULL;
    long v;

    if ((s == NULL) || (out == NULL))
    {
        return false;
    }

    v = strtol(s, &end, 10);
    if ((end == s) || (*end != '\0'))
    {
        return false;
    }

    *out = (int)v;
    return true;
}

static bool parse_float(const char *s, float *out)
{
    char *end = NULL;
    double v;

    if ((s == NULL) || (out == NULL))
    {
        return false;
    }

    v = strtod(s, &end);
    if ((end == s) || (*end != '\0'))
    {
        return false;
    }

    *out = (float)v;
    return true;
}

bool CLI_Parse(const char *line_in, cli_command_t *out)
{
    char  line[CLI_MAX_LINE + 1U];
    char *tok[4] = { 0 };
    int   ntok;

    if ((line_in == NULL) || (out == NULL))
    {
        return false;
    }

    (void)strncpy(line, line_in, CLI_MAX_LINE);
    line[CLI_MAX_LINE] = '\0';
    ToUpperCase(line);
    trim_inplace(line);

    if (line[0] == '\0')
    {
        out->primary   = CMD_NONE;
        out->secondary = SUB_NONE;
        out->has_int   = false;
        out->has_float = false;
        return true;
    }

    if ((strcmp(line, "X") == 0) || (strcmp(line, "EXIT") == 0))
    {
        out->primary   = CMD_EXIT;
        out->secondary = SUB_NONE;
        out->has_int   = false;
        out->has_float = false;
        return true;
    }

    if ((strcmp(line, "HELP") == 0) || (strcmp(line, "?") == 0))
    {
        out->primary   = CMD_HELP;
        out->secondary = SUB_NONE;
        out->has_int   = false;
        out->has_float = false;
        return true;
    }

    ntok = split_tokens(line, tok, 4);
    if (ntok <= 0)
    {
        return false;
    }

    out->primary   = CMD_NONE;
    out->secondary = SUB_NONE;
    out->has_int   = false;
    out->has_float = false;

    if (strcmp(tok[0], "CONFIG") == 0)
    {
        out->primary = CMD_CONFIG;
    }
    else if (strcmp(tok[0], "READ") == 0)
    {
        out->primary = CMD_READ;
    }
    else if (strcmp(tok[0], "RESET") == 0)
    {
        out->primary = CMD_RESET;
    }
    else if (strcmp(tok[0], "SET") == 0)
    {
        out->primary = CMD_SET;
    }
    else if (strcmp(tok[0], "HELP") == 0)
    {
        out->primary = CMD_HELP;
    }
    else if ((strcmp(tok[0], "EXIT") == 0) || (strcmp(tok[0], "X") == 0))
    {
        out->primary = CMD_EXIT;
    }
    else
    {
        return false;
    }

    switch (out->primary)
    {
        case CMD_CONFIG:
        {
            int   v;
            float f;

            if (ntok < 2)
            {
                return false;
            }

            if (strcmp(tok[1], "CHANNEL") == 0)
            {
                out->secondary = SUB_CHANNEL;
                if (ntok != 3)
                {
                    return false;
                }
                if (parse_int(tok[2], &v) == false)
                {
                    return false;
                }
                if ((v < 1) || (v > 4))
                {
                    return false;
                }
                out->has_int = true;
                out->ival    = v;
            }
            else if (strcmp(tok[1], "POWER") == 0)
            {
                out->secondary = SUB_POWER;
                if (ntok != 3)
                {
                    return false;
                }
                if (parse_float(tok[2], &f) == false)
                {
                    return false;
                }
                if ((f < 0.5f) || (f > 1.0f))
                {
                    return false;
                }
                out->has_float = true;
                out->fval      = f;
            }
            else
            {
                return false;
            }
        }
        break;

        case CMD_READ:
        {
            if (ntok < 2)
            {
                return false;
            }

            if (strcmp(tok[1], "CONFIG") == 0)
            {
                out->secondary = SUB_CONFIG;
                if (ntok != 2) { return false; }
            }
            else if (strcmp(tok[1], "DATA") == 0)
            {
                out->secondary = SUB_DATA;
                if (ntok != 2) { return false; }
            }
            else if (strcmp(tok[1], "ERRORS") == 0)
            {
                out->secondary = SUB_ERRORS;
                if (ntok != 2) { return false; }
            }
            else if (strcmp(tok[1], "OUTPUT") == 0)
            {
                out->secondary = SUB_OUTPUT;
                if (ntok != 2) { return false; }
            }
            else if (strcmp(tok[1], "DEFAULT") == 0)
            {
                out->secondary = SUB_DEFAULT;
                if (ntok != 2) { return false; }
            }
            else
            {
                return false;
            }
        }
        break;

        case CMD_RESET:
        {
            if (ntok != 2)
            {
                return false;
            }
            if (strcmp(tok[1], "ERRORS") != 0)
            {
                return false;
            }
            out->secondary = SUB_ERRORS;
        }
        break;

        case CMD_SET:
        {
            int v;

            if (ntok != 3)
            {
                return false;
            }

            if (strcmp(tok[1], "OUTPUT") == 0)
            {
                out->secondary = SUB_OUTPUT;
            }
            else if (strcmp(tok[1], "DEFAULT") == 0)
            {
                out->secondary = SUB_DEFAULT;
            }
            else
            {
                return false;
            }

            if (parse_int(tok[2], &v) == false)
            {
                return false;
            }
            if ((v != 0) && (v != 1))
            {
                return false;
            }
            out->has_int = true;
            out->ival    = v;
        }
        break;

        case CMD_HELP:
        case CMD_EXIT:
        {
            if (ntok != 1)
            {
                return false;
            }
        }
        break;

        default:
        {
            return false;
        }
    }

    return true;
}

/* Dispatcher: executes parsed command */
void CLI_Execute(const cli_command_t *cmd, cli_result_t *res)
{
    bool     ok;
    uint8_t  u8_val;
    float    f32_val;
    uint32_t u32_val;

    if ((cmd == NULL) || (res == NULL))
    {
        /* Nothing to do if arguments are invalid */
        return;
    }

    /* Initialize result with safe defaults */
    res->code   = CLI_RES_INVALID_CMD;
    res->detail = 0;
    res->u8     = 0U;
    res->f32    = 0.0f;
    res->u32    = 0UL;

    switch (cmd->primary)
    {
        case CMD_HELP:
        {
            res->code = CLI_RES_OK;
        }
        return;

        case CMD_EXIT:
        {
            res->code = CLI_RES_OK;
        }
        return;

        case CMD_CONFIG:
        {
            if ((cmd->secondary == SUB_CHANNEL) && (cmd->has_int == true))
            {
                ok = SetChannels((uint8_t)cmd->ival);
                if (ok == true)
                {
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else if ((cmd->secondary == SUB_POWER) && (cmd->has_float == true))
            {
                ok = SetPower(cmd->fval);
                if (ok == true)
                {
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else
            {
                res->code = CLI_RES_BAD_ARGS;
                return;
            }
        }

        case CMD_READ:
        {
            if (cmd->secondary == SUB_CONFIG)
            {
                /* Populate result payload (no printing here) */
                u8_val  = 0U;
                f32_val = 0.0f;

                ok = ReadChannels(&u8_val);
                if (ok == true)
                {
                    ok = ReadPower(&f32_val);
                    if (ok == true)
                    {
                        res->u8   = u8_val;
                        res->f32  = f32_val;
                        res->code = CLI_RES_OK;
                    }
                    else
                    {
                        res->code = CLI_RES_LINK_ERR;
                    }
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else if (cmd->secondary == SUB_DATA)
            {
                ok = ReadData();
                if (ok == true)
                {
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else if (cmd->secondary == SUB_ERRORS)
            {
                u32_val = 0UL;
                ok = ReadErrorflag(&u32_val);
                if (ok == true)
                {
                    res->u32  = u32_val;
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else if (cmd->secondary == SUB_OUTPUT)
            {
                u8_val = 0U;
                ok = ReadOutputState(&u8_val);
                if (ok == true)
                {
                    res->u8   = u8_val;
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else if (cmd->secondary == SUB_DEFAULT)
            {
                u8_val = 0U;
                ok = ReadDefaultState(&u8_val);
                if (ok == true)
                {
                    res->u8   = u8_val;
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else
            {
                res->code = CLI_RES_BAD_ARGS;
                return;
            }
        }

        case CMD_RESET:
        {
            if (cmd->secondary == SUB_ERRORS)
            {
                ok = ResetError();
                if (ok == true)
                {
                    res->code = CLI_RES_OK;
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else
            {
                res->code = CLI_RES_BAD_ARGS;
                return;
            }
        }

        case CMD_SET:
        {
            if ((cmd->secondary == SUB_OUTPUT) && (cmd->has_int == true))
            {
                if (cmd->ival != 0)
                {
                    u8_val = 1U;
                }
                else
                {
                    u8_val = 0U;
                }

                ok = WriteOutputState(u8_val);
                if (ok == true)
                {
                    res->u8   = u8_val;
                    res->code = CLI_RES_OK;
                    static const char ok[] = "OK\r\n";
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else if ((cmd->secondary == SUB_DEFAULT) && (cmd->has_int == true))
            {
                if (cmd->ival != 0)
                {
                    u8_val = 1U;
                }
                else
                {
                    u8_val = 0U;
                }

                ok = WriteDefaultState(u8_val);
                if (ok == true)
                {
                    res->u8   = u8_val;
                    res->code = CLI_RES_OK;
                    static const char ok[] = "OK\r\n";
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
                }
                else
                {
                    res->code = CLI_RES_LINK_ERR;
                }
                return;
            }
            else
            {
                res->code = CLI_RES_BAD_ARGS;
                return;
            }
        }

        default:
        {
            /* Unknown top-level command */
            res->code = CLI_RES_INVALID_CMD;
        }
        return;
    }
}

/* ================================
 * Presenter
 * ================================ */
static void print_error(cli_result_code_t code)
{
    const char *msg;

    msg = "ERR UNKNOWN\r\n";
    if (code == CLI_RES_OK)
    {
        msg = "OK\r\n";
    }
    else if (code == CLI_RES_INVALID_CMD)
    {
        msg = "ERR INVALID_CMD\r\n";
    }
    else if (code == CLI_RES_BAD_ARGS)
    {
        msg = "ERR BAD_ARGS\r\n";
    }
    else if (code == CLI_RES_RANGE_ERR)
    {
        msg = "ERR RANGE\r\n";
    }
    else if (code == CLI_RES_TIMEOUT)
    {
        msg = "ERR TIMEOUT\r\n";
    }
    else if (code == CLI_RES_BUSY)
    {
        msg = "ERR BUSY\r\n";
    }
    else if (code == CLI_RES_PROTOCOL_ERR)
    {
        msg = "ERR PROTOCOL\r\n";
    }
    else if (code == CLI_RES_LINK_ERR)
    {
        msg = "ERR LINK\r\n";
    }
    else
    {
        /* default already set */
    }

    (void)HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)strlen(msg), CLI_UART_TX_TIMEOUT_MS);
}

void CLI_PrintResult(const cli_command_t *cmd, const cli_result_t *res)
{
    char line[160];
    int  n;

    if ((cmd == NULL) || (res == NULL))
    {
        return;
    }

    if (cmd->primary == CMD_HELP)
    {
        CLI_PrintHelp();
        return;
    }

    if (cmd->primary == CMD_EXIT)
    {
        static const char ok[] = "OK\r\n";
        (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
        return;
    }

    if (res->code != CLI_RES_OK)
    {
        print_error(res->code);
        return;
    }

    switch (cmd->primary)
    {
        case CMD_CONFIG:
        {
            static const char ok[] = "OK\r\n";
            (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
        }
        break;

        case CMD_READ:
        {
            if (cmd->secondary == SUB_CONFIG)
            {
                if (res->u8 != 0xFFU)
                {
                    n = snprintf(line, sizeof(line), "CHANNELS: %u\r\n", (unsigned)res->u8);
                    if (n > 0)
                    {
                        (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                    }
                }
                if (res->f32 > 0.0f)
                {
                    n = snprintf(line, sizeof(line), "POWER: %.3f\r\n", (double)res->f32);
                    if (n > 0)
                    {
                        (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                    }
                }
                static const char ok[] = "OK\r\n";
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
            }
            else if (cmd->secondary == SUB_DATA)
            {
                static const char ok[] = "OK\r\n";
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
            }
            else if (cmd->secondary == SUB_ERRORS)
            {
                n = snprintf(line, sizeof(line), "ERRORS: 0x%08lX\r\nOK\r\n", (unsigned long)res->u32);
                if (n > 0)
                {
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                }
            }
            else if (cmd->secondary == SUB_OUTPUT)
            {
                n = snprintf(line, sizeof(line), "OUTPUT: %u\r\nOK\r\n", (unsigned)res->u8);
                if (n > 0)
                {
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                }
            }
            else if (cmd->secondary == SUB_DEFAULT)
            {
                n = snprintf(line, sizeof(line), "DEFAULT: %u\r\nOK\r\n", (unsigned)res->u8);
                if (n > 0)
                {
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                }
            }
            else
            {
                static const char ok[] = "OK\r\n";
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
            }
        }
        break;

        case CMD_RESET:
        {
            static const char ok[] = "OK\r\n";
            (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
        }
        break;

        case CMD_SET:
        {
            if (cmd->secondary == SUB_OUTPUT)
            {
                n = snprintf(line, sizeof(line), "OUTPUT:= %u\r\nOK\r\n", (unsigned)res->u8);
                if (n > 0)
                {
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                }
            }
            else if (cmd->secondary == SUB_DEFAULT)
            {
                n = snprintf(line, sizeof(line), "DEFAULT:= %u\r\nOK\r\n", (unsigned)res->u8);
                if (n > 0)
                {
                    (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, CLI_UART_TX_TIMEOUT_MS);
                }
            }
            else
            {
                static const char ok[] = "OK\r\n";
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
            }
        }
        break;

        default:
        {
            static const char ok[] = "OK\r\n";
            (void)HAL_UART_Transmit(&huart2, (uint8_t*)ok, (uint16_t)(sizeof(ok) - 1U), CLI_UART_TX_TIMEOUT_MS);
        }
        break;
    }
}

/* ================================
 * ManualMode (REPL)
 * ================================ */
bool ManualMode(void)
{
    char line[CLI_MAX_LINE + 1U];
    static const char banner[] =
        "\r\n*** DataLink CLI ***\r\n"
        "Type HELP for commands, X to exit.\r\n\r\n";

    (void)HAL_UART_Transmit(&huart2, (uint8_t*)banner, (uint16_t)(sizeof(banner) - 1U), CLI_UART_TX_TIMEOUT_MS);

    while(1)
    {
        static const char prompt[] = "> ";

        (void)HAL_UART_Transmit(&huart2, (uint8_t*)prompt, (uint16_t)(sizeof(prompt) - 1U), CLI_UART_TX_TIMEOUT_MS);

        if (CLI_ReadLine(line, (uint16_t)sizeof(line)) == false)
        {
            HAL_Delay(10U);
            continue;
        }

        ToUpperCase(line);

        if ((strcmp(line, "X") == 0) || (strcmp(line, "EXIT") == 0))
        {
            static const char bye[] = "Bye.\r\n";
            (void)HAL_UART_Transmit(&huart2, (uint8_t*)bye, (uint16_t)(sizeof(bye) - 1U), CLI_UART_TX_TIMEOUT_MS);
            return true;
        }

        if (line[0] == '\0')
        {
            continue;
        }
        else
        {
            cli_command_t cmd;
            cli_result_t  res;

            if (CLI_Parse(line, &cmd) == false)
            {
                static const char err[] = "ERR SYNTAX\r\n";
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)err, (uint16_t)(sizeof(err) - 1U), CLI_UART_TX_TIMEOUT_MS);
                continue;
            }

            if (cmd.primary == CMD_HELP)
            {
                CLI_PrintHelp();
                continue;
            }

            if (cmd.primary == CMD_EXIT)
            {
                static const char bye2[] = "Bye.\r\n";
                (void)HAL_UART_Transmit(&huart2, (uint8_t*)bye2, (uint16_t)(sizeof(bye2) - 1U), CLI_UART_TX_TIMEOUT_MS);
                return true;
            }

            CLI_Execute(&cmd, &res);
            CLI_PrintResult(&cmd, &res);
        }
    }
}
