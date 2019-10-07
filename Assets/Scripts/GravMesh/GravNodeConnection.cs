using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

public class GravNodeConnection 
{
    float m_RestDistance;
    GravNode m_GravNode1, m_GravNode2;

    public GravNodeConnection(GravNode gn1, GravNode gn2)
    {
        m_GravNode1 = gn1;
        m_GravNode2 = gn2;
        m_RestDistance = Vector3.Distance(gn1.gravNodeColliderParent.transform.position,
            gn2.gravNodeColliderParent.transform.position);
    }

    /* places two particles in rest distance from each other several
    times per cycle */
    public void SatisfyConstraint()
    {
        Vector3 gn1Togn2 = m_GravNode2.m_Position -
                               m_GravNode1.m_Position;

        float current_distance = Vector3.Distance(m_GravNode2.m_Position,
            m_GravNode1.m_Position); 
        Vector3 correctionVector = gn1Togn2*(1 - m_RestDistance/current_distance); 
        Vector3 correctionVectorHalf = correctionVector*0.5f;
        m_GravNode1.OffsetPos(correctionVectorHalf); 
        m_GravNode2.OffsetPos(-correctionVectorHalf); 	
    }
}
