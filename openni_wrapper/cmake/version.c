#include <XnVersion.h>
#include <stdio.h>

int main(int argc, char** argv)
{
  printf("%u.%u.%u", XN_MAJOR_VERSION, XN_MINOR_VERSION, XN_MAINTENANCE_VERSION);
}
