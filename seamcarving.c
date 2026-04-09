#include "seamcarving.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Dual‑gradient energy at pixel (y, x)
 double dual_gradient(const struct rgb_img *im, int y, int x){
    //helper
    int w = im->width;
    int h = im->height;

    // wrap‑around helper: maps index i to valid 0..w-1 or 0..h-1
    int xp1 = (x + 1) % w;
    int xm1 = (x - 1 + w) % w;
    int yp1 = (y + 1) % h;
    int ym1 = (y - 1 + h) % h;

    // differences in x
    int rx = (int)get_pixel(im, y, xp1, 0) - (int)get_pixel(im, y, xm1, 0);
    int gx = (int)get_pixel(im, y, xp1, 1) - (int)get_pixel(im, y, xm1, 1);
    int bx = (int)get_pixel(im, y, xp1, 2) - (int)get_pixel(im, y, xm1, 2);

    int delta_x2 = rx*rx + gx*gx + bx*bx;

    // differences in y
    int ry = (int)get_pixel(im, ym1, x, 0) - (int)get_pixel(im, yp1, x, 0);
    int gy = (int)get_pixel(im, ym1, x, 1) - (int)get_pixel(im, yp1, x, 1);
    int by = (int)get_pixel(im, ym1, x, 2) - (int)get_pixel(im, yp1, x, 2);

    int delta_y2 = ry*ry + gy*gy + by*by;

    return sqrt(delta_x2 + delta_y2);
}

void calc_energy(struct rgb_img *im, struct rgb_img **grad){
    //Part One
    int h = im->height;
    int w = im->width;

    // Create the gradient image
    create_img(grad, h, w);

    // For each pixel, compute energy and store in grad
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double e = dual_gradient(im, y, x);
            uint8_t e8 = (uint8_t)((int)e / 10);   // divide by 10, cast to uint8_t
            set_pixel(*grad, y, x, e8, e8, e8);   // same value in r,g,b
        }
    }
}

void dynamic_seam(struct rgb_img *grad, double **best_arr){
    //Part Two
    int h = grad->height;
    int w = grad->width;

    // Allocate 2D array: h rows, w columns
    *best_arr = (double *)calloc(h * w, sizeof(double)); // 1‑D storage, index by [y][x] = y*w + x
    double *best = *best_arr;

    // First row: base case
    for (int x = 0; x < w; x++) {
        best[0*w + x] = get_pixel(grad, 0, x, 0);
    }

    // Fill remaining rows with DP
    for (int y = 1; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y*w + x;
            double cur_e = get_pixel(grad, y, x, 0);

            // find minimum among three predecessors
            double min_prev = 1e20;
            for (int dx = -1; dx <= 1; dx++) {
                int xp = x + dx;
                if (xp >= 0 && xp < w) {
                    double val = best[(y-1)*w + xp];
                    if (val < min_prev) min_prev = val;
                }
            }

            best[idx] = cur_e + min_prev;
        }
    }
}

void recover_path(double *best, int height, int width, int **path){
    //Part Three

    // Allocate path: one int per row
    *path = (int *)malloc(height * sizeof(int));
    int *the_path = *path;

    // Index of last row in the 1D best array
    double min_val = best[(height-1)*width + 0];
    int min_col = 0;
    for (int x = 1; x < width; x++) {
        int idx = (height-1)*width + x;
        if (best[idx] < min_val) {
            min_val = best[idx];
            min_col = x;
        }
    }
    the_path[height-1] = min_col;

    // Backward: reconstruct path from bottom to top
    for (int y = height-2; y >= 0; y--) {
        int y_row = y*width;          // start of row y
        int next_x = the_path[y+1];   // column in row y+1
        int best_x = 0;
        double best_val = best[y_row + 0];

        for (int dx = -1; dx <= 1; dx++) {
            int xp = next_x + dx;
            if (xp >= 0 && xp < width) {
                if (best[y_row + xp] < best_val) {
                    best_val = best[y_row + xp];
                    best_x = xp;
                }
            }
        }
        the_path[y] = best_x;
    }
}

void remove_seam(struct rgb_img *src, struct rgb_img **dest, int *path){
    //Part Four
    int h = src->height;
    int w = src->width;

    // Create new image: same height, width-1
    create_img(dest, h, w-1);

    for (int y = 0; y < h; y++) {
        int skip_x = path[y];           // column to remove in this row
        int out_x = 0;                  // column in dest row
        for (int x = 0; x < w; x++) {
            if (x != skip_x) {
                uint8_t r = get_pixel(src, y, x, 0);
                uint8_t g = get_pixel(src, y, x, 1);
                uint8_t b = get_pixel(src, y, x, 2);
                set_pixel(*dest, y, out_x, r, g, b);
                out_x++;
            }
        }
    }
}