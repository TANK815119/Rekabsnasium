using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FPVImageProcessor_Human : MonoBehaviour
{
    [SerializeField] private RenderTexture sourceTexture;
    [SerializeField] private RenderTexture processedTexture;
    [SerializeField] private Transform cameraTrans;
    [SerializeField] private Transform POI;
    [SerializeField] private Camera cameraComp;

    void Start()
    {
        //if (TryGetComponent<Camera>(out Camera cameraComp))
        //{
        //}
        //else
        //{
        //    Debug.LogError(cameraTrans.gameObject.name + " does not have a camera componenet!");
        //}
    }

    void Update()
    {
        //make Texture2D drom the source RenderTexture
        Texture2D interTexture = RenderTextureToTexture2D(sourceTexture);

        //general code for a value 0 to 1 based on positon of camera and POI and POV
        //Vector2 screenPos = CalculatScreenSpace();
        Vector2 screenPos = cameraComp.WorldToViewportPoint(POI.position);

        if (screenPos.x >= 0f && screenPos.y >= 0f && screenPos.x <= 1f && screenPos.y <= 1f)
        {
            //make a little dot to reresent the screenspace positon of the POI for humans
            MarkTexture(interTexture, screenPos.x, screenPos.y);
        }
        else
        {
            //Debug.Log("POI not on screenspace");
        }

        //Convert the Texture2D to a RenderTexture ans assign
        RenderTexture outputTexture = Texture2DToRenderTexture(interTexture);
        Graphics.CopyTexture(outputTexture, processedTexture);
    }

    private Vector2 CalculatScreenSpace()
    {
        //find the angle between the target direction
        Vector3 targetDir = (POI.position - cameraTrans.position).normalized;
        float horiTheta = Vector3.SignedAngle(cameraTrans.forward, targetDir, cameraTrans.up);
        float vertTheta = Vector3.SignedAngle(cameraTrans.forward, targetDir, cameraTrans.right);

        //calculate x-value from theta
        float horiFOV = Camera.VerticalToHorizontalFieldOfView(cameraComp.fieldOfView, cameraComp.aspect);
        float horiProp = horiTheta / (horiFOV / 2f); //-1 to  1
        horiProp = (horiProp + 1f) / 2f; //from 0 to 1
        //horiProp *= Mathf.Cos(horiTheta * Mathf.Deg2Rad);

        //calculate y-value from theta
        float vertProp = vertTheta / (cameraComp.fieldOfView / 2f) * -1f; //-1 to 1
        vertProp = (vertProp + 1f) / 2f; //0 to 1
        //vertProp *= Mathf.Cos(vertTheta * Mathf.Deg2Rad);

        return new Vector2(horiProp, vertProp);
    }

    float AngleOnAxis(Vector3 a, Vector3 b, Vector3 axis)
    {
        // Project the vectors onto the plane orthogonal to the axis
        Vector3 projectedA = Vector3.ProjectOnPlane(a, axis);
        Vector3 projectedB = Vector3.ProjectOnPlane(b, axis);

        // Calculate the angle between the projected vectors
        float angle = Vector3.SignedAngle(projectedA, projectedB, axis);

        return angle;
    }

    private void MarkTexture(Texture2D texture, float x, float y)
    {
        if(x > 1 || y > 1 || x < 0 || y < 0)
        {
            Debug.LogError("screen sapce value is outside of 0-1 range");
        }

        // Calculate pixel coordinates
        int xPos = Mathf.RoundToInt(x * (texture.width - 1));
        int yPos = Mathf.RoundToInt(y * (texture.height - 1));

        // Set the pixel color to red
        texture.SetPixel(xPos, yPos, Color.red);

        // Apply the changes to the texture
        texture.Apply();
    }

    private Texture2D RenderTextureToTexture2D(RenderTexture renderTexture)
    {
        RenderTexture.active = renderTexture;
        Texture2D texture2D = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);
        texture2D.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;
        return texture2D;
    }

    private RenderTexture Texture2DToRenderTexture(Texture2D texture2D)
    {
        RenderTexture renderTexture = new RenderTexture(texture2D.width, texture2D.height, 0);
        Graphics.Blit(texture2D, renderTexture);
        return renderTexture;
    }
}