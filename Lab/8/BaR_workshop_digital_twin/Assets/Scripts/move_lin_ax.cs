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
File Name: move_lin_ax.cs
****************************************************************************/

// ------------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------- LIBRARIES --------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------------//

// -------------------- System -------------------- //
using System;
using System.Threading;
// -------------------- Unity -------------------- //
using UnityEngine;

public class move_lin_ax : MonoBehaviour
{
    // -------------------- GameObject -------------------- //
    // Link GameObject
    public GameObject linear_ax;
    // -------------------- Int -------------------- //
    // Linear axis motion control type
    // (0 -> Unity3D only, 1 -> OPC UA read current position, 2 -> OPC UA read targets {movement through Unity3D})
    public int type_of_control;
    // -------------------- Float -------------------- //
    public float target_position;
    public float velocity;
    private float actual_position;
    // -------------------- Vector3 -------------------- //
    private Vector3 initial_position;

    // ------------------------------------------------------------------------------------------------------------------------//
    // ------------------------------------------------ INITIALIZATION {START} ------------------------------------------------//
    // ------------------------------------------------------------------------------------------------------------------------//
    void Start()
    {
        // Initial position of the Link 1
        initial_position = linear_ax.transform.localPosition;
    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------------------ MAIN FUNCTION {Cyclic} ------------------------------------------------ //
    // ------------------------------------------------------------------------------------------------------------------------ //
    void Update()
    {
        switch (type_of_control)
        {
            case 0:
                {
                    // Control movement through public variables Unity3D (target position, velocity)
                    if (target_position <= 600 && target_position >= 0)
                    {
                        actual_position = Mathf.MoveTowards(actual_position, target_position, velocity * Time.deltaTime);
                        linear_ax.transform.localPosition = new Vector3(initial_position.x + actual_position, initial_position.y, initial_position.z);
                    }
                }
                break;

            case 1:
                {
                    // Reading the current position from the B&R OPC UA server
                    target_position = GlobalVariables_OPC_UA_client.linear_ax_pos[0];
                    linear_ax.transform.localPosition = new Vector3(initial_position.x + target_position, initial_position.y, initial_position.z);
                }
                break;

            case 2:
                {
                    // Control from current target position / speed (read variables from the B&R OPC UA server and move the axis with Unity3D)
                    target_position = GlobalVariables_OPC_UA_client.linear_ax_pos[1] / 10;
                    velocity = GlobalVariables_OPC_UA_client.linear_ax_vel / 10;

                    if (target_position <= 600 && target_position >= 0)
                    {
                        actual_position = Mathf.MoveTowards(actual_position, target_position, velocity * Time.deltaTime);
                        linear_ax.transform.localPosition = new Vector3(initial_position.x + actual_position, initial_position.y, initial_position.z);
                    }
                }
                break;
        }

        GlobalVariables_Main_Control.type_of_control = type_of_control;
    }
}
