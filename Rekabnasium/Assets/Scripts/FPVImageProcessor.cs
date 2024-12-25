using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FPVImageProcessor : MonoBehaviour
{
    [SerializeField] private RenderTexture sourceTexture;
    [SerializeField] private RenderTexture processedTexture;

    void Update()
    {
        //make Texture2D from SourceTexture
        Texture2D interTexture = RenderTextureToTexture2D(sourceTexture);

        //resize
        interTexture = ResizeTexture(interTexture, processedTexture.width, processedTexture.height);

        //Grayscale
        GrayscaleTexture(interTexture);

        //convert the Texture2D to RenderTexture
        RenderTexture outputTexture = Texture2DToRenderTexture(interTexture);
        Graphics.CopyTexture(outputTexture, processedTexture);
    }

    Texture2D ResizeTexture(Texture2D source, int width, int height)
    {
        Texture2D newTexture = new Texture2D(width, height);
        Color[] resizedPixels = new Color[width * height];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                // Rescale position (0 to 1) and sample original
                float xPercent = x / (float)width;
                float yPercent = y / (float)height;
                int originalX = Mathf.FloorToInt(xPercent * source.width);
                int originalY = Mathf.FloorToInt(yPercent * source.height);
                resizedPixels[y * width + x] = source.GetPixel(originalX, originalY);
            }
        }

        newTexture.SetPixels(resizedPixels);
        newTexture.Apply();
        return newTexture;
    }

    private void GrayscaleTexture(Texture2D texture)
    {
        Color[] pixels = texture.GetPixels();

        for (int i = 0; i < pixels.Length; i++)
        {
            float gray = pixels[i].r * 0.299f + pixels[i].g * 0.587f + pixels[i].b * 0.114f;
            pixels[i] = new Color(gray, gray, gray);
        }

        texture.SetPixels(pixels);
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