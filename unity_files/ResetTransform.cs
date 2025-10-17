using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ResetTransform : MonoBehaviour
{
    // Store the initial position and rotation
    private Vector3 initialLocalPosition;
    private Quaternion initialLocalRotation;

    // Start is called before the first frame update
    void Start()
    {
        // Save the initial transform values
        initialLocalPosition = transform.localPosition;
        initialLocalRotation = transform.localRotation;
    }

    // Update is called once per frame
    void Update()
    {
        // Check if the R key was pressed
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetToInitialTransform();
        }
    }

    // Resets the position and rotation
    void ResetToInitialTransform()
    {
        transform.localPosition = initialLocalPosition;
        transform.localRotation = initialLocalRotation;
    }
}
