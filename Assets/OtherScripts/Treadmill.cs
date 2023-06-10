using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Treadmill : MonoBehaviour
{
    // Start is called before the first frame update

    private float x;
    private float y;
    private float z;

    void Start()
    {
        x = this.transform.localPosition.x;
        y = this.transform.localPosition.y;
        z = this.transform.localPosition.z;
        
        if(this.name == "treadmill_slant")
        {
            y += 0.05f;
            this.transform.localPosition = new Vector3(x, y, z);
            this.transform.localRotation = new Quaternion(0, 0, 5, 1);
        } else
        {
            this.transform.localRotation = new Quaternion(0, 0, 0, 1);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (this.transform.localPosition.x <= 0f)
        {
            x = 16f;
        }
        this.transform.localPosition = new Vector3(x, y, z);
        x -= 0.0025f;
    }
}
