using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestShifter : MonoBehaviour
{

    GravGridBuilder m_Grid;
    // Start is called before the first frame update
    public float timerTime = 2.0f;
    float time;
    void Start()
    {
        m_Grid = GetComponent<GravGridBuilder>();
    }

    // Update is called once per frame
    void Update()
    {
        time += Time.deltaTime;
        if (time < timerTime)
            return;
        time = 0.0f;

        m_Grid.ShiftGrid(GravGridBuilder.GridShiftDirection.Down,1);
    }
}
