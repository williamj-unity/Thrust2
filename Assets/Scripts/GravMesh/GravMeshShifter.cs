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

        Vector3 pos = objectToKeepAtCenter.position;
        Vector3 center = gravGrid.GetMeshCenterWorld();
        Vector3 dir = (pos - center) * spacing;

        if (Vector3.Distance(center, pos) < radius)
            return;

        timer = 0;

        int stepsX = Mathf.FloorToInt(Mathf.Abs(dir.x));
        int stepY = Mathf.FloorToInt(Mathf.Abs(dir.y));

        if(dir.x > radius/2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Right, stepsX);
        }

        if (dir.y > radius / 2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Up, stepY);
        }

        if (dir.x < -radius / 2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Left, stepsX);
        }

        if (dir.y < -radius / 2.0f)
        {
            gravGrid.ShiftGrid(GravGridBuilder.GridShiftDirection.Down, stepY);
        }
    }
}
