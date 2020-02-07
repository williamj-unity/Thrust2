using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestNormalMatch : MonoBehaviour
{
    GravGridBuilder m_Grid;
    public float timerTime = 2.0f;

    IEnumerator Start()
    {
        m_Grid = GetComponent<GravGridBuilder>();

        while (true)
        {
            yield return new WaitForSeconds(timerTime);
            m_Grid.ShiftPlane();
        }
    }
}
