using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OssiclateSphereTest : MonoBehaviour
{

    public float range = 100;
    public float speed = 0.15f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(Mathf.Cos(Time.time * speed) * range, Mathf.Sin(Time.time * speed) * range, 0);
    }
}
