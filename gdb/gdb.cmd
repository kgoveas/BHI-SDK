set architecture EM
target remote | openocd -c "gdb_port pipe; log_output openocd.log" -f em7189_cjtag.cfg
