using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipParticleController : MonoBehaviour
{
    public ParticleSystem top;
    public ParticleSystem leftTop;
    public ParticleSystem leftBottom;
    public ParticleSystem rightTop;
    public ParticleSystem rightBottom;
    public ParticleSystem bottom;

    public void ShipRotationalDirectionInput(float direction)
    {
        if (direction > 0)
        {
            rightTop.Play();
            leftBottom.Play();
        }
        else if (direction < 0)
        {
            rightBottom.Play();
            leftTop.Play();
        }
    }

    public void ShipLinearDirectionInput(Vector2 direction)
    {
        if (direction.x > 0)
        {
            leftBottom.Play();
            leftTop.Play();
        }
        else if (direction.x < 0)
        {
            rightBottom.Play();
            rightTop.Play();
        }
        else
        {
            rightBottom.Stop();
            rightTop.Stop();
            leftBottom.Stop();
            leftTop.Stop();
        }
        
        if (direction.y > 0)
        {
            bottom.Play();
            top.Stop();
        }
        else if (direction.y < 0)
        {
            top.Play();
            bottom.Stop();
        }
        else
        {
            bottom.Stop();
            top.Stop();
        }
    }
}
