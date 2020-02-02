using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravMeshShifter : MonoBehaviour
{
    public GravGridBuilder gravGrid;
    public Transform objectToKeepAtCenter;
    public float radius = 1.0f;

    float spacing;

    public float time = 0.25f;
    float timer;
    void Start()
    {
        spacing = gravGrid.spacing;
    }
    void Update()
    {
        timer += Time.deltaTime;
        if (timer < time)
            return;

        timer = 0;

        Vector3 pos = objectToKeepAtCenter.position;
        Vector3 center = gravGrid.GetMeshCenterWorld();
        Vector3 dir = (pos - center) * spacing;

        if (Vector3.Distance(center, pos) < radius)
            return;

        if(dir.x > radius/2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Right);
        }

        if (dir.y > radius / 2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Up);
        }

        if (dir.x < -radius / 2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Left);
        }

        if (dir.y < -radius / 2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Down);
        }
    }
}
