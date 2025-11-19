// Sample1.cpp : A simple C++ example using DataServer.dll COM object.
//
// Instantiates DataServer, loads a configuration file, and acquires data.

#include "stdafx.h"
#include <objbase.h>
#include <atlsafe.h> // include for CComSafeArray
#include <fstream>
#include <string>
#include <vector>

// import data server 
// TODO: change path to DataServer.dll
#import "C:\dev\DataServer\DataServer.dll"

// Global file stream for CSV
std::ofstream csvFile;

void InitializeCSV() {
    csvFile.open("data_output_002.csv");
    if (!csvFile.is_open()) {
        printf("Error: Could not open CSV file for writing!\n");
        return;
    }

    // Write CSV headers for 16 channels
    csvFile << "Scan";
    for (int i = 1; i <= 16; i++) {
        csvFile << ",Channel_" << i;
    }
    csvFile << std::endl;
    printf("CSV file initialized successfully with 16 channel columns.\n");
}

void CloseCSV() {
    if (csvFile.is_open()) {
        csvFile.close();
        printf("CSV file closed successfully.\n");
    }
}

void ProcessData(DataServerLib::IPlatePtr plate, int start_scan, int end_scan)
{
    CComSafeArray<float> fx;
    fx.Attach(plate->arrFx(start_scan, end_scan));

    CComSafeArray<float> fy;
    fy.Attach(plate->arrFy(start_scan, end_scan));

    CComSafeArray<float> fz;
    fz.Attach(plate->arrFz(start_scan, end_scan));

    CComSafeArray<float> ax;
    ax.Attach(plate->arrAx(start_scan, end_scan));

    CComSafeArray<float> ay;
    ay.Attach(plate->arrAy(start_scan, end_scan));

    for (int i = fx.GetLowerBound(); i <= fx.GetUpperBound(); i++) {
        printf("[%d] %f, %f, %f, %f, %f\n", start_scan + i, fx[i], fy[i], fz[i], ax[i], ay[i]);
    }
}

void GetData(DataServerLib::IPlatePtr plate, int start_scan, int end_scan) {
    float fx = plate->GetFx(start_scan);
    float fy = plate->GetFy(start_scan);
    float fz = plate->GetFz(start_scan);
    float ax = plate->GetAx(start_scan);
    float ay = plate->GetAy(start_scan);
    float time = plate->GetTime(1000, start_scan);
    printf("[%d] %f, %f, %f, %f, %f, %f\n", start_scan, time, fx, fy, fz, ax, ay);
}

int _tmain(int argc, _TCHAR* argv[])
{
    CoInitialize(NULL);

    // Initialize CSV file at the start
    InitializeCSV();

    // Build Scope for COM CoInitialize/CoUnitialize
    {
        // local variable to track completion.
        ULONG prevscan(0);

        // Create the server object
        DataServerLib::IDataServerPtr server;
        server.CreateInstance(L"DataServer.DataServer");
        if (server == NULL) {
            CloseCSV();
            return -1;
        }

        try
        {
            // Get DAQ object, set Configuration file, board type, and Instacal Board Number
            // TODO: 
            // - change path to config.xml if needed. (parameter 1)
            // - set board type (parameter 2)
            // - set board number from InstaCal (parameter 3)
            DataServerLib::IDaqControlPtr daq;
            daq = server->CreateDaqObject(L"C:\\dev\\Westview-Capstone\\XML\\config.xml", DataServerLib::brdKistler5695A1, 0);

            // set amplifier to on 
            daq->MeasureOn();

            // read offets if enabled
            daq->ReadOffsets(DataServerLib::rngBIP10VOLTS);

            // Get the collection of devices
            DataServerLib::IDeviceCollectionPtr dc = daq->GetDeviceCollection();
            if (dc == NULL) {
                printf("Device collection is NULL\n");
                CloseCSV();
                return -1;
            }

            // Look up device by name (from the XML file)
            // The generic device interface provides Time, and Voltage Data.
            // the device name is the user defined name in XML file. Ex: <Name>Plate 1</Name>
            // TODO: Set device name to match a plate in the XML config file.
            DataServerLib::IDevicePtr device = dc->get_DeviceByName(L"Plate 1");
            if (device == NULL) {
                printf("First Device is NULL\n");
                CloseCSV();
                return -1;
            }
            // Get interface to plate
            // The plate pointer servers Fx, Fy, Fz, Mx, My, Mz, Ax, Ay, Time, and Voltage Data.
            // Or, use IKistlerPlatePtr to get raw force Fx12, Fx34, Fy14, Fy23, Fz1, Fz2, Fz3, Fz4 Data.
            DataServerLib::IPlatePtr plate = device;
            if (plate == NULL) {
                printf("First Plate is NULL\n");
                CloseCSV();
                return -1;
            }
            // prepare data acqusition, and wait for software trigger event
            // Start (long rate_per_channel, long samples_per_channel, enum trigger_option, enum daq_range)
            daq->Start(1000, 10000, DataServerLib::trigImmediate, DataServerLib::rngBIP10VOLTS);

            int counter = 0;
            // loop while running
            while (daq->Running) {
                //wprintf(L"run status : %d...\n", daq->Running);
                // yield to other threads, let them do some work
                Sleep(50);

                // check current amount available
                long prevscan = 0;
                long thisscan = daq->LastAvailableScan;

                if (thisscan > prevscan) {
                    // Vector to store all channel values for this scan
                    //std::vector<float> channelValues(16);

                    //// Collect all channel data first
                    //for (int i = 1; i <= 16; i++) {
                    //    float output = daq->GetSampleCorrected(i, thisscan);
                    //    channelValues[i - 1] = output; // Store in 0-based index
                    //    //wprintf(L"Scan %d: Channel %i Corrected Sample: %f\n", thisscan, i, output);
                    //}

                    //// Write one row to CSV with all 16 channels
                    //if (csvFile.is_open()) {
                    //    csvFile << thisscan;
                    //    for (int i = 0; i < 16; i++) {
                    //        csvFile << "," << channelValues[i];
                    //    }
                    //    csvFile << std::endl;
                    //}
					ProcessData(plate, prevscan, thisscan);

                    prevscan = thisscan;
                    //wprintf(L"processed %d...\n", thisscan);
                }

                counter++;
            }

            // call stop (although, it should already be stopped)
            daq->Stop();

            // place plate into reset
            daq->MeasureOff();

        }
        catch (_com_error e)
        {
            wprintf((LPCTSTR)e.Description());
        }
    }

    // Close CSV file at the end
    CloseCSV();
    CoUninitialize();
    return 0;
}