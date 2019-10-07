using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipController : MonoBehaviour
{
    
    // TODO:: this will eventually need to be refactored into an input manager that assigns actions based 
    // on different controller types that are connected. I've added it here now to how this should be done
    // ,albeit in a more generic way, in the future. 
    public enum ShipControllerType
    {
        Keyboard,
        Controller
    }

    public float m_KeyboardLinearThrustScalar = 2.0f;
    public float m_KeyboardAngularThrustScalar = 4.0f;

    public float m_LinearDampingCoef = 2.0f;
    public float m_AngularDampingCoef = 2.0f;

    public ShipControllerType m_ControllerType = ShipControllerType.Keyboard;
    
    // Start is called before the first frame update
    private Rigidbody2D shipRigidBody;

    public float drag;
    public float mass;
    public float gravity;

    public ParticleSystem _particleSystem;

    public bool dampersOn { get; private set; } = true;
    
    void Start()
    {
        shipRigidBody = gameObject.AddComponent<Rigidbody2D>();
        shipRigidBody.drag = drag;
        shipRigidBody.mass = mass;
        shipRigidBody.gravityScale = gravity;
    }

    void AddLinearThrust(float thrustValue, Vector2 normalizedDirection)
    {
        shipRigidBody.AddRelativeForce(normalizedDirection * thrustValue);
        if (thrustValue > 0.1f)
        {
        }
    }

    void AddRotationalThrust(float thrustValue, float direction)
    {
        shipRigidBody.AddTorque(thrustValue * direction);
    }

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
            
            if (Input.GetKey(KeyCode.A))
            {
                thrustDir += Vector2.left;
            }
            
            if (Input.GetKey(KeyCode.S))
            {
                thrustDir += Vector2.down;
            }
            
            if (Input.GetKey(KeyCode.D))
            {
                thrustDir += Vector2.right;
            }
            
            AddLinearThrust(m_KeyboardLinearThrustScalar, thrustDir.normalized);

            float rotationalDirection = 0.0f;
            if (Input.GetKey(KeyCode.Q))
            {
                rotationalDirection += 1.0f;
            }
            
            if (Input.GetKey(KeyCode.E))
            {
                rotationalDirection -= 1.0f;
            }
            
            AddRotationalThrust(m_KeyboardAngularThrustScalar, rotationalDirection);

            if (Input.GetKeyDown(KeyCode.Z))
            {
                dampersOn = !dampersOn;
            }
        }


        if (dampersOn)
        {
            AddRotationalThrust(-shipRigidBody.angularVelocity * m_AngularDampingCoef, 1.0f);
            
            
            AddLinearThrust(shipRigidBody.velocity.magnitude * m_LinearDampingCoef,  -transform.InverseTransformDirection(shipRigidBody.velocity).normalized);
        }
    }
}
