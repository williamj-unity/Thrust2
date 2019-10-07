using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravNodeCollider : MonoBehaviour
{
    public Action<float> affectorCollisionEnter;

    public Action affectorCollisionExit;
    // Start is called before the first frame update

    private float maxAffectorMass = Single.NegativeInfinity;
    private float m_Spacing = 1.0f;
    void Start()
    {
        var collider = gameObject.AddComponent<SphereCollider>();
        collider.radius = m_Spacing;
    }

    public void SetSpacing(float spacing)
    {
        m_Spacing = spacing;
    }

    private void OnTriggerEnter(Collider other)
    {
        GravNodeAffector affectorOut;
        if (other.TryGetComponent<GravNodeAffector>(out affectorOut))
        {
            maxAffectorMass = Mathf.Max(affectorOut.mass, maxAffectorMass);
            affectorCollisionEnter(maxAffectorMass);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        GravNodeAffector affectorOut;
        if (other.TryGetComponent<GravNodeAffector>(out affectorOut))
        {
            affectorCollisionExit();
        }
    }
}
