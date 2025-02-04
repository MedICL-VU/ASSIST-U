import cv2
import numpy as np
import matplotlib.pyplot as plt


def main():
    img = cv2.imread("test_img.png", cv2.IMREAD_GRAYSCALE)

    # Visualize original image
    plt.figure(figsize=(10, 8))
    plt.subplot(2, 2, 1)
    plt.imshow(img, cmap='gray')
    plt.title("1) Original Image")
    plt.axis('off')

    # ------------------------------------------------
    # 2. Color Inversion
    # ------------------------------------------------
    des = cv2.bitwise_not(img)

    plt.subplot(2, 2, 2)
    plt.imshow(des, cmap='gray')
    plt.title("2) Inverted Image")
    plt.axis('off')

    # -----------------------------------------
    # 3. Find Contours
    # -----------------------------------------
    contour, hier = cv2.findContours(des, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    contourplot = cv2.drawContours(img, contour, 0, (0, 255, 0), 5)


    plt.subplot(2, 2, 3)
    plt.imshow(contourplot, cmap='gray')
    plt.title(f"3) Labeled Contours\n(num_labels = {len(contour)})")
    plt.axis('off')

    # -----------------------------------------
    # 4. Clean Image
    # -----------------------------------------
    cv2.drawContours(des, contour, 0, (0, 255, 0), -1)
    gray = cv2.bitwise_not(des) # invert back

    plt.subplot(2, 2, 4)
    plt.imshow(gray, cmap='gray')
    plt.title("4) Cleaned Image")
    plt.axis('off')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
