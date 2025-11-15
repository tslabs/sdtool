
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

int main(void)
{
  const struct shell *sh = shell_backend_uart_get_ptr();

  if (sh)
  {
    while (!shell_ready(sh))
    {
      k_sleep(K_MSEC(10));
    }

    shell_execute_cmd(sh, "log disable");
  }
  
  return 0;
}
