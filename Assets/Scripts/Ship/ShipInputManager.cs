using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipInputManager : MonoBehaviour
{
    
    public Action<Vector2> shipLinearDirectionInput;
    public Action<float> shipRotationalDirectionInput;

    public Action damperEvent;
    
    // TODO:: this will eventually need to be refactored into an input manager that assigns actions based 
    // on different controller types that are connected. I've added it here now to how this should be done
    // ,albeit in a more generic way, in the future. 
    public enum ShipControllerType
    {
        Keyboard,
        Controller
    }
    
    public ShipControllerType m_ControllerType = ShipControllerType.Keyboard;

    // Update is called once per frame
    void Update()
    {
        if (m_ControllerType == ShipControllerType.Keyboard)
        {
            Vector2 thrustDir = Vector2.zero;
            if (Input.GetKey(KeyCode.W))
            {
                thrustDir += Vector2.up;
            }
            
            if (Input.GetKey(KeyCode.Q))
            {
                thrustDir += Vector2.left;
            }
            
            if (Input.GetKey(KeyCode.S))
            {
                thrustDir += Vector2.down;
            }
            
            if (Input.GetKey(KeyCode.E))
            {
                thrustDir += Vector2.right;
            }

            shipLinearDirectionInput(thrustDir);
            
            float rotationalDirection = 0.0f;
            if (Input.GetKey(KeyCode.A))
            {
                rotationalDirection += 1.0f;
            }
            
            if (Input.GetKey(KeyCode.D))
            {
                rotationalDirection -= 1.0f;
            }

            shipRotationalDirectionInput(rotationalDirection);

            if (Input.GetKeyDown(KeyCode.Z))
            {
                damperEvent();
            }
        }

    }
}
