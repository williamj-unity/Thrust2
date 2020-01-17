using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipCannon : MonoBehaviour, IShipWeapon
{
    public GameObject laserPrefab;
    public Transform[] t;
    int currentCannonTransform = 0;
    int cannonCount;

    public float weasponsfiringRate;
    float shootTime;
    float firingRatePerSecond;
    bool canShoot;

    void Start()
    {
        firingRatePerSecond = 1.0f / weasponsfiringRate;
        cannonCount = t.Length;
    }
    public void Shoot(Vector3 velocity)
    {
        if(firingRatePerSecond <= shootTime)
        {
            shootTime = 0.0f;
            Transform spawnTransform = t[currentCannonTransform++ % cannonCount];
            GameObject go = Instantiate(laserPrefab, spawnTransform.position, spawnTransform.rotation);
            go.GetComponent<CannonBullet>().SetVeloctiy(velocity);
        }
    }

    public void Update()
    {
        shootTime += Time.deltaTime;
    }
}
