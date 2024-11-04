using ConsoleTables;
using System.IO.Ports;

try {
    if (args == null || args.Length != 1)
    {
        throw new InvalidOperationException("Provide a serial port.\n\tUsage: StimJimBIST <serial port>\n\te.g. StimJimBIST COM3");
    }

    var portName = args[0];

    Console.WriteLine("Running StimJim BIST.");

    var globalPass = true;
    var port = new SerialPort(portName);
    port.Open();

    int[] testVoltages = [-9000, 0, 9000];

    const double VoltageGain = 1.505;
    const int VoltageTolerancemV = 50;
    const int CurrentToleranceuA = 50;

    // Clear initial content
    while (port.BytesToRead > 0)
    {
        var buff = new byte[port.BytesToRead];
        port.Read(buff, 0, port.BytesToRead);
    }

    Console.WriteLine("Testing voltage mode.");
    Console.WriteLine();

    port.WriteLine("M0,0");
    port.ReadLine();
    port.WriteLine("M1,0");
    port.ReadLine();

    for (int chan = 0; chan < 2; chan++)
    {
        Console.WriteLine($"Channel {chan}:");
        var table = new ConsoleTable("Test V.", "Meas V.", "Err. mV.", "Pass / Fail", "Test uA.", "Meas uA.", "Err. uA", "Pass / Fail");

        for (int i = 0; i < testVoltages.Length; i++)
        {
            var vTest = testVoltages[i];
            var iTest = 1e6 * vTest / 1000 / VoltageGain / 3e3;

            port.WriteLine($"V{chan},{vTest}");
            port.ReadLine();

            // Verify that we are within 100 mV
            port.WriteLine($"E{chan},0");
            var voltageReport = port.ReadLine();
            var vMeas = float.Parse(voltageReport.Split(['(', ')'])[1].TrimEnd(['m', 'V']));

            port.WriteLine($"E{chan},1");
            var currentReport = port.ReadLine();
            var iMeas = float.Parse(currentReport.Split(['(', ')'])[1].TrimEnd(['u', 'A']));

            var vError = vMeas - vTest;
            var iError = iMeas - iTest;

            var vPass = Math.Abs(vError) < VoltageTolerancemV;
            var iPass = Math.Abs(iError) < CurrentToleranceuA;

            globalPass = globalPass && vPass && iPass;

            table.AddRow((vTest / 1000.0).ToString("0.00"), (vMeas / 1000.0).ToString("0.00"), vError, FormatPassFail(vPass), iTest.ToString("0.00"), iMeas.ToString("0.00"), iError.ToString("0.00"), FormatPassFail(iPass));

        }
        table.Write(Format.Minimal);
    }

    Console.WriteLine("Testing current mode.");
    Console.WriteLine();

    port.WriteLine("M0,1");
    port.ReadLine();
    port.WriteLine("M1,1");
    port.ReadLine();

    testVoltages = [-1000, 1000];
    double[] targVoltage = [-10000, 10000];


    for (int chan = 0; chan < 2; chan++)
    {
        Console.WriteLine($"Channel {chan}:");
        var table = new ConsoleTable("Pol.", "Meas V.", "Err. mV.", "Pass / Fail", "Meas uA.", "Err. uA", "Pass / Fail");

        for (int i = 0; i < testVoltages.Length; i++)
        {
            var vTest = testVoltages[i];
            var iTest = 1e6 * vTest / 1000 / VoltageGain / 3e3;

            port.WriteLine($"V{chan},{vTest}");
            port.ReadLine();

            // Verify that we are within 100 mV
            port.WriteLine($"E{chan},0");
            var voltageReport = port.ReadLine();
            var vMeas = float.Parse(voltageReport.Split(['(', ')'])[1].TrimEnd(['m', 'V']));

            port.WriteLine($"E{chan},1");
            var currentReport = port.ReadLine();
            var iMeas = float.Parse(currentReport.Split(['(', ')'])[1].TrimEnd(['u', 'A']));

            var vError = vMeas - targVoltage[i];
            var iError = iMeas - 0;

            var vPass = Math.Abs(vError) < VoltageTolerancemV;
            var iPass = Math.Abs(iError) < CurrentToleranceuA;

            globalPass = globalPass && vPass && iPass;

            table.AddRow(vTest > 0 ? "+" : "-", (vMeas / 1000.0).ToString("0.00"), vError, FormatPassFail(vPass), iMeas.ToString("0.00"), iError.ToString("0.00"), FormatPassFail(iPass));

        }
        table.Write(Format.Minimal);
    }

    Console.WriteLine($"GLOBAL RESULT: {FormatPassFail(globalPass)}");

    port.Close();

    string FormatPassFail(bool pass)
    {
        return pass ? "PASS" : "FAIL";
    }

} catch (Exception e)
{
    Console.WriteLine(e.Message);
}