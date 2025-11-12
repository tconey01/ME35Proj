import cv2
import numpy as np

def create_player_heatmap(image_path):
    """Create a heatmap highlighting player positions"""
    
    img = cv2.imread(image_path)
    
    # Convert to HSV to isolate non-green areas (players)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Define green field color range
    lower_green = np.array([35, 30, 30])
    upper_green = np.array([85, 255, 255])
    
    # Create mask: 255 where NOT green (players), 0 where green (field)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    player_areas = cv2.bitwise_not(green_mask)
    
    # Blur to create heat effect (spread the heat around players)
    heatmap = cv2.GaussianBlur(player_areas, (21, 21), 0)
    
    # Apply JET colormap (blue->green->yellow->red)
    heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
    
    # Blend: 50% original image + 50% heatmap
    result = cv2.addWeighted(img, 0.5, heatmap_colored, 0.5, 0)
    
    # Save result
    cv2.imwrite("output_heatmap_final.jpg", result)
    print("✅ Heatmap saved as output_heatmap_final.jpg")
    
    return result

# Run
create_player_heatmap("football.jpeg")
