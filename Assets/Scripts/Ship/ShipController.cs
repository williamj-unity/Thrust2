using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipController : MonoBehaviour
{
    public float m_KeyboardLinearThrustScalar = 2.0f;
    public float m_KeyboardAngularThrustScalar = 4.0f;

    public float m_LinearDampingCoef = 2.0f;
    public float m_AngularDampingCoef = 2.0f;

    // Start is called before the first frame update
    private Rigidbody2D shipRigidBody;

    public float drag;
    public float mass;
    public float gravity;

    public bool dampersOn { get; private set; } = true;

    public ShipInputManager InputManager;

    ShipParticleController particleController;
    IShipWeapon currentWeopon;
    bool applyLinearDamping;
    bool applyRotationalDamping;

    void Start()
    {
        shipRigidBody = gameObject.AddComponent<Rigidbody2D>();
        shipRigidBody.drag = drag;
        shipRigidBody.mass = mass;
        shipRigidBody.gravityScale = gravity;

        InputManager.damperEvent += () => { dampersOn = !dampersOn; };
        InputManager.shipLinearDirectionInput += vector2 => AddLinearThrust(m_KeyboardAngularThrustScalar, vector2);
        InputManager.shipRotationalDirectionInput += f => AddRotationalThrust(m_KeyboardAngularThrustScalar, f);
        InputManager.allRotationalControlsUpEvent += b => { applyRotationalDamping = b; };
        InputManager.allLinearControlsUpEvent += b => { applyLinearDamping = b; };

        InputManager.shootEvent += () => Shoot();
        ChangeWeopon(GetComponent<ShipCannon>());

        particleController = GetComponent<ShipParticleController>();
    }

    public void ChangeWeopon(IShipWeapon weapon)
    {
        currentWeopon = weapon;
    }

    void Shoot()
    {
        currentWeopon.Shoot(shipRigidBody.velocity);
    }

    void AddLinearThrust(float thrustValue, Vector2 normalizedDirection)
    {
        particleController.ShipLinearDirectionInput(normalizedDirection);
        shipRigidBody.AddRelativeForce(normalizedDirection * thrustValue);
    }

    void AddRotationalThrust(float thrustValue, float direction)
    {
        particleController.ShipRotationalDirectionInput(direction);
        shipRigidBody.AddTorque(thrustValue * direction);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        shipRigidBody.drag = drag;
        if (dampersOn)
        {
            if(applyRotationalDamping)
                AddRotationalThrust(-shipRigidBody.angularVelocity * m_AngularDampingCoef, 1.0f);

            if(applyLinearDamping)
                AddLinearThrust(shipRigidBody.velocity.magnitude * m_LinearDampingCoef, -transform.InverseTransformDirection(shipRigidBody.velocity).normalized);
        }
    }
}
