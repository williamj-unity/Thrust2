using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravMeshNormalMatcher : MonoBehaviour
{

    public GravGridBuilder ggBuilder;

    // Start is called before the first frame update
    IEnumerator Start()
    {
        while(true)
        {
            yield return new WaitForSeconds(0.5f);
            ggBuilder.ArrangeAroundNormal(transform);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
