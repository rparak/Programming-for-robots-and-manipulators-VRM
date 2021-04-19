/****************************************************************************
MIT License
Copyright(c) 2020 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************
Author   : Roman Parak
Email    : Roman.Parak @outlook.com
Github   : https://github.com/rparak
File Name: br_data_processing.cs
****************************************************************************/

// ------------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------- LIBRARIES --------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------------//

// -------------------- System -------------------- //
using System;
using System.Threading;
// -------------------- Unity -------------------- //
using UnityEngine;
// -------------------- OPCUA -------------------- //
using Opc.Ua;
using Opc.Ua.Client;
using Opc.Ua.Configuration;

// -------------------- Class {Global Variable} -> Main Control -------------------- //
public static class GlobalVariables_Main_Control
{
    // String //
    public static string[] opcua_config = new string[2];
    // Bool // 
    public static bool enable_r;
    // -------------------- Int -------------------- //
    // Linear axis motion control type
    // (0 -> Unity3D only, 1 -> OPC UA read current position, 2 -> OPC UA read targets {movement through Unity3D})
    public static int type_of_control;
}

public static class GlobalVariables_OPC_UA_client
{
    // -------------------- XYZ Manipulator -------------------- //
    //  NodeId  //
    public static NodeId[] linear_ax_pos_node = new NodeId[2] { "ns=6;s=::axCtrl_T:ctrl_lin_ax_0.write.dT.position", "ns=6;s=::axCtrl_T:ctrl_lin_ax_0.write.position" };
    public static NodeId linear_ax_vel_node   = "ns=6;s=::axCtrl_T:ctrl_lin_ax_0.write.velocity";
    //  Float  //
    public static float[] linear_ax_pos = new float[2];
    public static float linear_ax_vel;
}


public class br_data_processing : MonoBehaviour
{
    // -------------------- ApplicationConfiguration -------------------- //
    private ApplicationConfiguration client_configuration_r = new ApplicationConfiguration();
    // -------------------- EndpointDescription -------------------- //
    private EndpointDescription client_end_point_r;
    // -------------------- Session -------------------- //
    private Session client_session_r;
    // -------------------- Thread -------------------- //
    private Thread opcua_client_r_Thread;
    // -------------------- Int -------------------- //
    private int main_br_state = 0;

    // ------------------------------------------------------------------------------------------------------------------------//
    // ------------------------------------------------ INITIALIZATION {START} ------------------------------------------------//
    // ------------------------------------------------------------------------------------------------------------------------//
    void Start()
    {
        // ------------------------ Initialization { B&R/SMC Digital Twin {Control Robot} - OPC UA Read Data } ------------------------//
        // PLC IP Address
        GlobalVariables_Main_Control.opcua_config[0] = "127.0.0.1";
        // OPC UA Port Number
        GlobalVariables_Main_Control.opcua_config[1] = "4840";
        // Control -> Start {Read OPCUA data}
        GlobalVariables_Main_Control.enable_r = true;

    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------------------ MAIN FUNCTION {Cyclic} ------------------------------------------------ //
    // ------------------------------------------------------------------------------------------------------------------------ //
    private void Update()
    {
        switch (main_br_state)
        {
            case 0:
                {
                    // ------------------------ Wait State {Disconnect State} ------------------------//
                    if (GlobalVariables_Main_Control.type_of_control > 0)
                    {
                        // Control -> Start {Read OPCUA data}
                        GlobalVariables_Main_Control.enable_r = true;

                        // Initialization threading block
                        opcua_client_r_Thread = new Thread(() => OPCUa_r_thread_function(GlobalVariables_Main_Control.opcua_config[0], GlobalVariables_Main_Control.opcua_config[1]));
                        opcua_client_r_Thread.IsBackground = true;
                        // Start threading block
                        opcua_client_r_Thread.Start();

                        // go to connect state
                        main_br_state = 1;
                    }
                }
                break;
            case 1:
                {
                    // ------------------------ Data Processing State {Connect State} ------------------------//
                    if (GlobalVariables_Main_Control.type_of_control == 0)
                    {
                        // Control -> Start {Read OPCUA data}
                        GlobalVariables_Main_Control.enable_r = false;
                        // Abort threading block {OPCUA -> read data}
                        if (opcua_client_r_Thread.IsAlive == true)
                        {
                            opcua_client_r_Thread.Abort();
                        }

                        if (opcua_client_r_Thread.IsAlive == false)
                        {
                            // go to initialization state {wait state -> disconnect state}
                            main_br_state = 0;
                        }
                    }
                }
                break;
        }
    }


    // ------------------------------------------------------------------------------------------------------------------------//
    // -------------------------------------------------------- FUNCTIONS -----------------------------------------------------//
    // ------------------------------------------------------------------------------------------------------------------------//

    // -------------------- Abort Threading Blocks -------------------- //
    void OnApplicationQuit()
    {
        try
        {
            // Stop - threading while
            GlobalVariables_Main_Control.enable_r  = false;

            // Abort threading block
            if (opcua_client_r_Thread.IsAlive == true)
            {
                opcua_client_r_Thread.Abort();
            }

        }
        catch (Exception e)
        {
            // Destroy all
            Destroy(this);
        }
        finally
        {
            // Destroy all
            Destroy(this);
        }
    }

    void OPCUa_r_thread_function(string ip_adr, string port_adr)
    {
        if (GlobalVariables_Main_Control.enable_r == true)
        {
            // OPCUa client configuration
            client_configuration_r = opcua_client_configuration();
            // Establishing communication
            client_end_point_r = CoreClientUtils.SelectEndpoint("opc.tcp://" + ip_adr + ":" + port_adr, useSecurity: false, operationTimeout: 5000);
            // Create session
            client_session_r = opcua_create_session(client_configuration_r, client_end_point_r);
        }

        // Threading while {read data}
        while (GlobalVariables_Main_Control.enable_r)
        {
            // ---------- FLOAT ---------- //
            // Position of the Linear Axis (Actual {0}, Target {1})
            GlobalVariables_OPC_UA_client.linear_ax_pos[0] = float.Parse(client_session_r.ReadValue(GlobalVariables_OPC_UA_client.linear_ax_pos_node[0]).ToString(), System.Globalization.CultureInfo.InvariantCulture);
            GlobalVariables_OPC_UA_client.linear_ax_pos[1] = float.Parse(client_session_r.ReadValue(GlobalVariables_OPC_UA_client.linear_ax_pos_node[1]).ToString(), System.Globalization.CultureInfo.InvariantCulture);
            // Velocity of the Linear Axis
            GlobalVariables_OPC_UA_client.linear_ax_vel = float.Parse(client_session_r.ReadValue(GlobalVariables_OPC_UA_client.linear_ax_vel_node).ToString(), System.Globalization.CultureInfo.InvariantCulture);
            // Thread Sleep {2 ms}
            Thread.Sleep(2);
        }
    }

    // ------------------------ OPCUa Client {Application -> Configuration (STEP 1)} ------------------------//
    ApplicationConfiguration opcua_client_configuration()
    {
        // Configuration OPCUa Client {W/R -> Data}
        var config = new ApplicationConfiguration()
        {
            // Initialization (Name, Uri, etc.)
            ApplicationName = "OPCUa_AS", // OPCUa AS (Automation Studio B&R)
            ApplicationUri = Utils.Format(@"urn:{0}:OPCUa_AS", System.Net.Dns.GetHostName()),
            // Type -> Client
            ApplicationType = ApplicationType.Client,
            SecurityConfiguration = new SecurityConfiguration
            {
                // Security Configuration - Certificate
                ApplicationCertificate = new CertificateIdentifier { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\MachineDefault", SubjectName = Utils.Format(@"CN={0}, DC={1}", "OPCUa_AS", System.Net.Dns.GetHostName()) },
                TrustedIssuerCertificates = new CertificateTrustList { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\UA Certificate Authorities" },
                TrustedPeerCertificates = new CertificateTrustList { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\UA Applications" },
                RejectedCertificateStore = new CertificateTrustList { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\RejectedCertificates" },
                AutoAcceptUntrustedCertificates = true,
                AddAppCertToTrustedStore = true
            },
            TransportConfigurations = new TransportConfigurationCollection(),
            TransportQuotas = new TransportQuotas { OperationTimeout = 5000 },
            ClientConfiguration = new ClientConfiguration { DefaultSessionTimeout = 60000 },
            TraceConfiguration = new TraceConfiguration()
        };
        config.Validate(ApplicationType.Client).GetAwaiter().GetResult();
        if (config.SecurityConfiguration.AutoAcceptUntrustedCertificates)
        {
            config.CertificateValidator.CertificateValidation += (s, e) => { e.Accept = (e.Error.StatusCode == StatusCodes.BadCertificateUntrusted); };
        }

        var application = new ApplicationInstance
        {
            ApplicationName = "OPCUa_AS",
            ApplicationType = ApplicationType.Client,
            ApplicationConfiguration = config
        };
        application.CheckApplicationInstanceCertificate(false, 2048).GetAwaiter().GetResult();

        return config;
    }

    // ------------------------ OPCUa Client {Application -> Create Session (STEP 2)} ------------------------//
    Session opcua_create_session(ApplicationConfiguration client_configuration, EndpointDescription client_end_point)
    {
        return Session.Create(client_configuration, new ConfiguredEndpoint(null, client_end_point, EndpointConfiguration.Create(client_configuration)), false, "", 5000, null, null).GetAwaiter().GetResult();
    }

}
