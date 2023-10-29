#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <regex>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gtk/gtk.h>

#define log(smth) std::cout << smth << std::endl;



GdkPixbuf* matToPixbuf(const cv::Mat& mat) {
    cv::Mat rgbMat;
    cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB); // OpenCV uses BGR by default, convert it to RGB

    // Create a GdkPixbuf from the OpenCV Mat data
    return gdk_pixbuf_new_from_data(
        reinterpret_cast<const guchar*>(rgbMat.data),
        GDK_COLORSPACE_RGB,
        FALSE,
        8,
        rgbMat.cols,
        rgbMat.rows,
        rgbMat.step,
        nullptr,
        nullptr
    );
}

// Function to draw the image on the GtkDrawingArea
gboolean drawImage(GtkWidget* widget, cairo_t* cr, gpointer userData) {
    GdkPixbuf* pixbuf = static_cast<GdkPixbuf*>(userData);
    if (pixbuf != nullptr) {
        gdk_cairo_set_source_pixbuf(cr, pixbuf, 0, 0);
        cairo_paint(cr);
    }
    return FALSE;
}


static void activate (GtkApplication* app, gpointer user_data) {
    GtkWidget *window;

    cv::Mat image;
    log("loading");
    image = cv::imread("./Cameras/1/Cal/20231013_190003.jpg");
    log("done");

    window = gtk_application_window_new (app);
    gtk_window_set_title (GTK_WINDOW (window), "Window");
    gtk_window_set_default_size (GTK_WINDOW (window), 500, 500);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), nullptr);

    log("default window");

/*
    // Create a GtkDrawingArea to display the image
    GtkWidget* drawingArea = gtk_drawing_area_new();
    gtk_container_add(GTK_CONTAINER(window), drawingArea);

    log("drawing area");

    // Convert the OpenCV Mat to GdkPixbuf
    GdkPixbuf* pixbuf = matToPixbuf(image);

    log("mat to pix buff");

    // Set the GdkPixbuf as user data to use in the drawing callback
    g_object_set_data(G_OBJECT(drawingArea), "pixbuf", pixbuf);

    log("GdkPixbuf");

    // Set the drawing callback for the GtkDrawingArea
    g_signal_connect(G_OBJECT(drawingArea), "draw", G_CALLBACK(drawImage), pixbuf);

    log("signal");
*/
    gtk_widget_show_all(window);

    log("dead");

    }

int main (int argc, char **argv) {
  GtkApplication *app;
  int status;

  app = gtk_application_new ("org.gtk.example", G_APPLICATION_DEFAULT_FLAGS);
  log("not done");
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  log("done");
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  return status;
}