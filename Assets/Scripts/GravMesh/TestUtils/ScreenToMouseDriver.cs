using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenToMouseDriver : MonoBehaviour
{
    // Start is called before the first frame update
    private Camera cam;
    float z;
    void Start()
    {
        cam = Camera.main;
        z = -cam.transform.position.z;
    }

    // Update is called once per frame
    void Update()
    {
        Vector2 mousePos = Input.mousePosition;
        Vector3 screenToWorldPoint = cam.ScreenToWorldPoint(new Vector3(mousePos.x, mousePos.y, z));
        transform.position = screenToWorldPoint;
    }
}
