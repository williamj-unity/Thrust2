using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestShifter : MonoBehaviour
{

    GravGridBuilder m_Grid;
    // Start is called before the first frame update
    public float timerTime = 2.0f;

    public int iters = 4;

    public GravGridBuilder.GridShiftDirection[] gridShiftDirections = new GravGridBuilder.GridShiftDirection[] {
        GravGridBuilder.GridShiftDirection.Right,
        GravGridBuilder.GridShiftDirection.Down,
        GravGridBuilder.GridShiftDirection.Left,
        GravGridBuilder.GridShiftDirection.Up };

    int m = 0;
    IEnumerator Start()
    {
        m_Grid = GetComponent<GravGridBuilder>();

        while(true)
        {
            for (int i = 0; i < iters; i++)
            {
                yield return new WaitForSeconds(timerTime);
                m_Grid.ShiftGrid(gridShiftDirections[m % gridShiftDirections.Length]);
            }
            m++;
        }
    }
}
