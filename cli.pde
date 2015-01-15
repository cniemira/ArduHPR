int8_t cli_cmd_mode (uint8_t argc, const Menu::arg *argv)
{
	int mode;
    if (argc != 2)
    {
    	hal.console->printf_P(PSTR("Usage: mode MODE\n"));
    }

    mode = (int)argv[1].i;
    if (mode <= HPR_MODE_RECOVERED)
    {
    	hpr_state = mode;
    }

    return 0;
}

bool cli_cmd_preprompt(void)
{
	hal.console->printf_P(PSTR("[status=%u] [mode=%u]\n"), hpr_status, hpr_state);
	return true;
}

const struct Menu::command cli_main_menu_commands[] PROGMEM = {
		{"mode",		cli_cmd_mode},
};

MENU2(cli_main_menu, "[main", cli_main_menu_commands, cli_cmd_preprompt);

static void cli_init(void)
{
	return;
}


static uint8_t cli_return_count;
static void cli_update(void)
{
	int c = hal.console->read();
	if (c > 0)
	{
		if('\r' == c || '\n' == c)
		{
			if (cli_return_count++ > 2)
			{
				hal.console->printf_P(PSTR("Starting CLI...\n\n"));
				cli_return_count = 0;
				cli_main_menu.run();
			}
		}
	}
}
