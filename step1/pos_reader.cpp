// ==============================
// STEP1: Position Reader
// ==============================

// Standard headers
#include <stdlib.h>
#include <stdio.h>

// Dynamixel SDK header
#include <dynamixel_sdk/dynamixel_sdk.h>

// Keyboard input related (Linux)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

// Control table adress (XM430-W210, XC330-T288)
#define ADDR_PRESENT_POSITION    116

// Data byte length
#define LEN_PRESENT_POSITION     4

// Protocal version
#define PROTOCAL_VERSION         2.0

// Settings
#define DXL1_ID                  1
#define DXL2_ID                  2
#define DXL3_ID                  3
#define DXL4_ID                  4
#define DXL5_ID                  5
#define DXL6_ID                  6
#define DXL7_ID                  7
#define DXL8_ID                  8

#define NUMBER_OF_DXL            8

#define BAUDRATE                 4000000
#define DEVICENAME               "/dev/ttyUSB0"

#define ESC_ASCII_VALUE          0x1b

#define OFFSET                   2048              // Offset for Dynamixel motor positions -> radians
#define SCALE_FACTOR             3.141592/2048     // Scale factor for Dynamixel motor positions -> radians


// For keyboard input
int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }

    return 0;
}

// Main
int main()
{
    // Initialize PortHandler, PacketHandler, and GroupSyncRead instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCAL_VERSION);
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        return 0;
    }

    // Add parameter storage for Dynamixel present position values
    int dxl_ids[NUMBER_OF_DXL] = {DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID};
    int dxl_comm_result = COMM_TX_FAIL;
    bool dxl_addparam_result = false;
    bool dxl_getdata_result = false;
    uint8_t dxl_error = 0;                           // Dynamixel error
    int32_t dxl_positions_dxl[NUMBER_OF_DXL] = {};   // Dynamixel raw positions
    float dxl_positions_rad[NUMBER_OF_DXL] = {};     // Dynamixel positions in radians

    for (int i = 0; i < NUMBER_OF_DXL; i++)
    {
        dxl_addparam_result = groupSyncRead.addParam(dxl_ids[i]);
        if (dxl_addparam_result == true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead addparam succeeded!\n", dxl_ids[i]);
        }
        else
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dxl_ids[i]);
            return 0;
        }
    }

    printf("\n");
    printf("Beginning to read joint positions...\n");
    printf("Press any key to continue and ESC to exit! \n");
    printf("\n");

    // Read Positions
    while(1)
    {
        if (getch() ==  ESC_ASCII_VALUE)
        {
            break;
        }

        // Syncread current position
        dxl_comm_result = groupSyncRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        
        // Check if groupsyncread data is available
        for (int i = 0; i < NUMBER_OF_DXL; i++)
        {
            dxl_getdata_result = groupSyncRead.isAvailable(dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_ids[i]);
                return 0;
            }
        }

        // Get current positions
        for (int i = 0; i < NUMBER_OF_DXL; i++)
        {
            dxl_positions_dxl[i] = groupSyncRead.getData(dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            dxl_positions_rad[i] = (dxl_positions_dxl[i]-OFFSET)*SCALE_FACTOR;
            printf("[ID:%03d] PresPos:%.4f \n", dxl_ids[i],  dxl_positions_rad[i]);
        }

        printf("\n");
    }

    // Close port
    portHandler->closePort();

    return 0;

}

