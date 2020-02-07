using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class GravNodeCollider : MonoBehaviour
{
    public Action<float> affectorCollisionEnter;

    public Action affectorCollisionExit;

    public Func<float3> GetNodeTransformFunc;
    // Start is called before the first frame update

    private float maxAffectorMass = Single.NegativeInfinity;
    private float m_Spacing = 1.0f;
    bool m_Moveable = false;

    SortedSet<GravNodeAffector> collidedGravNodeAffectors = new SortedSet<GravNodeAffector>(new GravAffectorComparer());
    void Start()
    {
        var collider = gameObject.AddComponent<SphereCollider>();
        collider.radius = m_Spacing;
    }

    public void SetSpacing(float spacing)
    {
        m_Spacing = spacing;
    }

    public void SetMoveable(bool b)
    {
        m_Moveable = b;
        if (!b)
        {
            collidedGravNodeAffectors.Clear();
            affectorCollisionExit();
        }
    }

    public float3 GetNodeTransform()
    {
        return GetNodeTransformFunc.Invoke();
    }

    private void OnTriggerEnter(Collider other)
    {
        GravNodeAffector affectorOut;
        if (other.TryGetComponent<GravNodeAffector>(out affectorOut))
        {
            if (!m_Moveable)
                return;
            if (!collidedGravNodeAffectors.Contains(affectorOut))
                collidedGravNodeAffectors.Add(affectorOut);
            //else
                //Debug.LogErrorFormat("Two objects {0} and {1} collided with the same grav node and have the same mass. Change the mass slightly on one to avoid problems", affectorOut.name, collidedGravNodeAffectors.Max.name);

            affectorCollisionEnter(collidedGravNodeAffectors.Max.mass);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        GravNodeAffector affectorOut;
        if (other.TryGetComponent<GravNodeAffector>(out affectorOut) )
        {
            if (!m_Moveable)
                return;

            if (collidedGravNodeAffectors.Contains(affectorOut))
                collidedGravNodeAffectors.Remove(affectorOut);
            else
                return;

            if (collidedGravNodeAffectors.Count > 0)
                affectorCollisionEnter(collidedGravNodeAffectors.Max.mass);
            else
                affectorCollisionExit();
        }
    }
}

public class GravAffectorComparer : IComparer<GravNodeAffector>
{
    public int Compare(GravNodeAffector x, GravNodeAffector y)
    {
        if (x.mass > y.mass)
            return 1;
        else if (x.mass < y.mass)
            return -1;
        else
        {
            return 0;
        }
    }
}

