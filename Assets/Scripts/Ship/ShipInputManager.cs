using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipInputManager : MonoBehaviour
{
    
    public Action<Vector2> shipLinearDirectionInput;
    public Action<float> shipRotationalDirectionInput;

    public Action damperEvent;
    public Action shootEvent;
    public Action<bool> allLinearControlsUpEvent;
    public Action<bool> allRotationalControlsUpEvent;

    Vector2 thrustDir = Vector2.zero;
    float rotationalDirection = 0.0f;

    // TODO:: this will eventually need to be refactored into an input manager that assigns actions based 
    // on different controller types that are connected. I've added it here now to how this should be done
    // ,albeit in a more generic way, in the future. 
    public enum ShipControllerType
    {
        Keyboard,
        Controller
    }
    
    public ShipControllerType m_ControllerType = ShipControllerType.Keyboard;


    void FixedUpdate()
    {
        shipLinearDirectionInput(thrustDir);
        shipRotationalDirectionInput(rotationalDirection);
    }

    // Update is called once per frame
    void Update()
    {
        thrustDir = Vector2.zero;
        rotationalDirection = 0.0f;
        if (m_ControllerType == ShipControllerType.Keyboard)
        {
            bool allLinearControlsUp = true;
            bool allRotationalControlsUp = true;

            if (Input.GetKey(KeyCode.W))
            {
                thrustDir += Vector2.up;
                allLinearControlsUp = false;
            }
            
            if (Input.GetKey(KeyCode.Q))
            {
                thrustDir += Vector2.left;
                allLinearControlsUp = false;
            }

            if (Input.GetKey(KeyCode.S))
            {
                thrustDir += Vector2.down;
                allLinearControlsUp = false;
            }

            if (Input.GetKey(KeyCode.E))
            {
                thrustDir += Vector2.right;
                allLinearControlsUp = false;
            }


            if (Input.GetKey(KeyCode.A))
            {
                rotationalDirection += 1.0f;
                allRotationalControlsUp = false;
            }

            if (Input.GetKey(KeyCode.D))
            {
                rotationalDirection -= 1.0f;
                allRotationalControlsUp = false;
            }

            allLinearControlsUpEvent(allLinearControlsUp);
            allRotationalControlsUpEvent(allRotationalControlsUp);

            if (Input.GetKeyDown(KeyCode.Z))
            {
                damperEvent();
            }
            

            if(Input.GetKey(KeyCode.Space))
            {
                shootEvent();
            }
        }

    }
}
