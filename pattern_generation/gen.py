# Modified version from Cheng, Minghao
import numpy as np
import cv2
from PIL import Image

used_random_marker = []

def gen_one_marker(dict, markerID, noise_marker:bool, marker_size=6):
    num_pixels = marker_size + 4
    bits = np.zeros(shape = (num_pixels, num_pixels), dtype = bool)
    
    marker_bits = dict[markerID].reshape((marker_size, marker_size))
    if noise_marker:
        marker_bits.ravel()
    bits[2:2+marker_size,2:2+marker_size] = marker_bits
    
    # set boundary to true
    bits[0,:] = True
    bits[num_pixels - 1,:] = True
    bits[:,0] = True
    bits[:, num_pixels - 1] = True
    return bits

def genRandom_table(table_width, table_height, pixel_size):
    size_w = int(table_width/pixel_size)
    size_h = int(table_height/pixel_size)
    bits = np.random.random_integers(0,1,size=(size_w, size_h))
    bits = bits == 1
    return bits

def bits_to_img(bits, table_size, dpp, colour0, colour1):
    table_width, table_height = table_size
    img = np.zeros(shape = (table_width * dpp, table_height * dpp,3), dtype=np.uint8)
    for (row, row_val) in enumerate(bits):
        for (col, val) in enumerate(row_val):
            if (val):
                img[row * dpp : row * dpp + dpp, col * dpp : col * dpp + dpp] = colour1
            else:
                img[row * dpp : row * dpp + dpp, col * dpp : col * dpp + dpp] = colour0

    return img

def get_aruco_bits_from_id(marker_id, dictionary_name=cv2.aruco.DICT_APRILTAG_36h11, marker_size=6):
    # Get the dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_name)

    # Generate the marker image (larger than marker_size to allow thresholding)
    img_size = 200
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, img_size)

    # Resize to the desired internal marker size (remove black border)
    # OpenCV adds a 1-cell black border around the marker grid, so crop accordingly
    cell_size = img_size // (marker_size + 2)
    cropped = marker_img[cell_size: -cell_size, cell_size: -cell_size]

    # Resize to a known size for easy thresholding
    cropped_resized = cv2.resize(cropped, (marker_size, marker_size), interpolation=cv2.INTER_NEAREST)

    # Convert to boolean bits (thresholding)
    bits = cropped_resized > 127
    return bits.astype(bool)

def main():
    marker_size = 6
    dict_file = {
        i: get_aruco_bits_from_id(
            i, 
            marker_size=marker_size
        ).astype(np.uint8).ravel() for i in range(100)
    }
    
    # integers, change could cause problem, not tested
    table_width = 120 # cm
    table_height = 100 # cm
    pixel_size = 1 # cm
    num_pixels = 12 # 12*12 marker

    table = genRandom_table(table_width, table_height, pixel_size)
    # put 9*9 markers on the table, the central 7*7 are real markers
    offset_w = 1
    offset_h = 2

    n_blocks=6
    rows = 10
    cols = 8
    print(table.shape)
    count = 0
    for row in range(0, rows, 1):
        for col in range(0, cols, 1):
            count += 1
            if (row == 0 or row == rows-1 or col == 0 or col == cols-1):
                marker = gen_one_marker(dict_file, 0, noise_marker=True)
            
            marker = gen_one_marker(dict_file, row * cols + col, noise_marker=False)
            tw1 = row * num_pixels + offset_w + 1 
            tw2 = (row + 1) * num_pixels + offset_w - 1
            th1 = col * num_pixels + offset_h + 1
            th2 = (col + 1) * num_pixels + offset_h - 1
            print(count, row, col, tw1, tw2, th1, th2)
            table[tw1: tw2, th1 : th2] = marker

    # dots per cm, for printing
    # use 5px per mm
    # dpcm = 10*5
    # 300dpi = 300/2.54 dpcm
    dpcm = 118
    # dpcm = 10
    colour_false = [0, 0, 0]
    # colour_true = [179, 97, 46] # RGB
    # colour_true = [46, 97, 179,] # BGR
    # colour_true = [65, 115, 200,] # BGR
    colour_true = [98, 123, 189,] # BGR
    # colour_true = [200, 115, 65,] # RGB

    img_table = bits_to_img(
        table, (table_width, table_height), dpcm, colour_false, colour_true
    )
    # generate files for printing
    print(img_table.shape)
    # cv2.imwrite("table.bmp", img_table)
    # img_pil = Image.fromarray(img_table)
    # img_pil.convert('CMYK').save('table.jpg', quality=100)
    # img_pil.save('table.pdf', quality=100)
    





if __name__ == "__main__":
    main()