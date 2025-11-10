#include "CyberdogInterface.h"
#include "signal.h"

std::shared_ptr<CyberdogInterface> io;

void signal_callback_handler(int sign)
{
    io->Stop();
    (void) sign;
}

int main()
{
    signal(SIGINT,signal_callback_handler);
    io = std::make_shared<CyberdogInterface>(500);
    io->Spin();
    
    return 0;
}