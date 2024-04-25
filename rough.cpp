// g++ DDA.cpp -o file -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32

// DDA
#include <iostream>
#include <graphics.h>
using namespace std;

void DDA(int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;

    int steps;
    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = dx / (float)steps;
    float y_inc = dy / (float)steps;

    float x = x1;
    float y = y1;

    for (int i = 0; i <= steps; i++)
    {
        putpixel(round(x), round(y), YELLOW);
        x += x_inc;
        y += y_inc;
        delay(100);
    }
}

int main()
{
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);

    int x0, x1, y0, y1;
    cout << "Enter the coordiinates of the end points of the line ";
    cin >> x0 >> y0 >> x1 >> y1;

    DDA(x0, y0, x1, y1);
    delay(500);
    getch();
    closegraph();
    return 0;
}

// Bresenham's Line
#include <bits/stdc++.h>
#include <graphics.h>
using namespace std;

void Bresenham(int x1, int y1, int x2, int y2)
{
    int m = (y2 - y1) / (x2 - x1);
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);

    if ((m >= 0 and m < 1))
    {
        int p = 2 * dy - dx;
        int x = x1;
        int y = y1;

        for (int i = 0; i < dx; i++)
        {
            x++;

            if (p < 0)
            {
                p = p + 2 * dy;
            }

            else
            {
                y++;
                p = p + 2 * (dy - dx);
            }

            putpixel(x, y, YELLOW);
            delay(100);
        }
    }

    else if (m > -1 and m <= 0)
    {
        int p = 2 * dy - dx;
        int x = x1;
        int y = y1;

        for (int i = 0; i < dx; i++)
        {
            x++;

            if (p < 0)
            {
                p = p + 2 * dy;
            }

            else
            {
                y--;
                p = p + 2 * (dy - dx);
            }

            putpixel(x, y, YELLOW);
            delay(100);
        }
    }

    else if (m >= 1)
    {
        int p = 2 * dx - dy;
        int x = x1;
        int y = y1;

        for (int i = 0; i < dy; i++)
        {
            y++;

            if (p < 0)
            {
                p = p + 2 * dx;
            }

            else
            {
                x++;
                p = p + 2 * (dx - dy);
            }

            putpixel(x, y, YELLOW);
            delay(100);
        }
    }

    else if (m <= -1)
    {
        int p = 2 * dx - dy;
        int x = x1;
        int y = y1;

        for (int i = 0; i < dy; i++)
        {
            y++;

            if (p < 0)
            {
                p = p + 2 * dx;
            }

            else
            {
                x--;
                p = p + 2 * (dx - dy);
            }

            putpixel(x, y, YELLOW);
            delay(100);
        }
    }
}

int main()
{
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);
    int x1, x2, y1, y2;
    cout << "Enter";
    cin >> x1 >> x2 >> y1 >> y2;
    Bresenham(x1, y1, x2, y2);
    delay(500);
    getch();
    closegraph();
    return 0;
}

// midpoint line drawing
#include <bits/stdc++.h>
using namespace std;
#include <graphics.h>

void midPoint(int X1, int Y1, int X2, int Y2)
{

    int dx = X2 - X1;
    int dy = Y2 - Y1;

    if (dy <= dx)
    {
        int d = dy - (dx / 2);
        int x = X1, y = Y1;

        cout << x << "," << y << "\n";

        while (x < X2)
        {
            x++;
            if (d < 0)
                d = d + dy;
            else
            {
                d += (dy - dx);
                y++;
            }

            cout << x << "," << y << "\n";
        }
    }

    else if (dx < dy)
    {

        int d = dx - (dy / 2);
        int x = X1, y = Y1;

        cout << x << "," << y << "\n";

        while (y < Y2)
        {
            y++;

            if (d < 0)
                d = d + dx;
            else
            {
                d += (dx - dy);
                x++;
            }

            cout << x << "," << y << "\n";
        }
    }
}

int main()
{
    int gd = DETECT, gm;

    initgraph(&gd, &gm, NULL);

    int X1 = 2, Y1 = 2, X2 = 8, Y2 = 5;
    midPoint(X1, Y1, X2, Y2);
    delay(500);
    getch();
    closegraph();
    return 0;
}

// midpoint circle
#include <bits/stdc++.h>
#include <graphics.h>
using namespace std;

void drawCircle(int xc, int yc, int x, int y)
{
    putpixel(xc + x, yc + y, YELLOW);
    putpixel(xc - x, yc + y, YELLOW);
    putpixel(xc + x, yc - y, YELLOW);
    putpixel(xc - x, yc - y, YELLOW);
    putpixel(xc + y, yc + x, YELLOW);
    putpixel(xc - y, yc + x, YELLOW);
    putpixel(xc + y, yc - x, YELLOW);
    putpixel(xc - y, yc - x, YELLOW);
}

void midpointC(int xc, int yc, int r)
{
    int x = 0;
    int y = r;
    int p = 1 - r;

    while (y < x)
    {
        x++;
        if (p < 0)
        {
            p = p + 2 * x + 3;
        }

        else
        {
            y--;
            p += 2 * (x - y) + 5;
        }

        drawCircle(xc, yc, x, y);
    }
}

int main()
{
    int gd = DETECT, gm;
    initgraph(&gd, &gm, "");
    int xc, yc, r;
    cout << "Enter";
    cin >> xc >> yc >> r;
    midpointC(xc, yc, r);
    midpointC(xc, yc, r);
    delay(500);
    getch();
    closegraph();
    return 0;
}

// bresenham circle
#include <bits/stdc++.h>
#include <graphics.h>
using namespace std;

void drawCircle(int xc, int yc, int x, int y)
{
    putpixel(xc + x, yc + y, YELLOW);
    putpixel(xc - x, yc + y, YELLOW);
    putpixel(xc + x, yc - y, YELLOW);
    putpixel(xc - x, yc - y, YELLOW);
    putpixel(xc + y, yc + x, YELLOW);
    putpixel(xc - y, yc + x, YELLOW);
    putpixel(xc + y, yc - x, YELLOW);
    putpixel(xc - y, yc - x, YELLOW);
}

void bresenhamC(int xc, int yc, int r)
{
    int x = 0;
    int y = r;
    int p = 3 - 2 * r;

    while (y < x)
    {
        x++;
        if (p < 0)
        {
            p = p + 4 * x + 6;
        }

        else
        {
            y--;
            p += 4 * (x - y) + 10;
        }

        drawCircle(xc, yc, x, y);
        delay(100);
    }
}

int main()
{
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);
    int xc, yc, r;
    cout << "Enter";
    cin >> xc >> yc >> r;
    bresenhamC(xc, yc, r);
    delay(500);
    getch();
    closegraph();
    return 0;
}

// midpoint ellipse
#include <bits/stdc++.h>
#include <graphics.h>
using namespace std;

void ellipse(int xc, int yc, int rx, int ry)
{
    int x = 0, y = ry;
    int p;
    int rx2 = rx * rx;
    int ry2 = ry * ry;
    int px = 0;
    int py = 2 * rx2 * y;

    p = round(ry2 - (rx2 * ry) + (0.25 * rx2));
    while (px < py)
    {
        putpixel(xc + x, yc + y, YELLOW);
        putpixel(xc - x, yc + y, YELLOW);
        putpixel(xc + x, yc - y, YELLOW);
        putpixel(xc - x, yc - y, YELLOW);

        x++;
        px += 2 * ry2;
        if (p < 0)
        {
            p += ry2 + px;
        }

        else
        {
            y--;
            py -= 2 * rx2;
            p += ry2 + px - py;
        }
    }

    // region 2
    p = round(ry2 * (x + 0.5) * (x + 0.5) + rx2 * (y - 1) * (y - 1) - rx2 * ry2);
    while (y >= 0)
    {
        putpixel(xc + x, yc + y, YELLOW);
        putpixel(xc - x, yc + y, YELLOW);
        putpixel(xc + x, yc - y, YELLOW);
        putpixel(xc - x, yc - y, YELLOW);

        y--;
        py -= 2 * rx2;
        if (p > 0)
        {
            p += rx2 - py;
        }

        else
        {
            x++;
            px += 2 * ry2;
            p += rx2 + px - py;
        }
    }
}

int main()
{
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);
    int xc, yc, rx, ry;
    cout << "Enter";
    cin >> xc >> yc >> rx >> ry;
    ellipse(xc, yc, rx, ry);
    delay(500);
    getch();
    closegraph();
    return 0;
}

// cohen sutherland
#include <graphics.h>
#include <iostream>
using namespace std;

const int INSIDE = 0; // 0000
const int LEFT = 1;   // 0001
const int RIGHT = 2;  // 0010
const int BOTTOM = 4; // 0100
const int TOP = 8;    // 1000

int computeCode(double x, double y, double x_min, double y_min, double x_max, double y_max)
{
    int code = INSIDE;

    if (x < x_min)
        code |= LEFT;
    else if (x > x_max)
        code |= RIGHT;
    if (y < y_min)
        code |= BOTTOM;
    else if (y > y_max)
        code |= TOP;

    return code;
}

void cohenSutherlandClip(double x1, double y1, double x2, double y2, double x_min, double y_min, double x_max, double y_max)
{
    int code1 = computeCode(x1, y1, x_min, y_min, x_max, y_max);
    int code2 = computeCode(x2, y2, x_min, y_min, x_max, y_max);
    bool accept = false;

    while (true)
    {
        if ((code1 == 0) && (code2 == 0))
        {
            accept = true;
            break;
        }
        else if (code1 & code2)
        {
            break;
        }
        else
        {
            double x, y;
            int outcode;
            outcode = code1 ? code1 : code2;

            if (outcode & TOP)
            {
                x = x1 + (x2 - x1) * (y_max - y1) / (y2 - y1);
                y = y_max;
            }
            else if (outcode & BOTTOM)
            {
                x = x1 + (x2 - x1) * (y_min - y1) / (y2 - y1);
                y = y_min;
            }
            else if (outcode & RIGHT)
            {
                y = y1 + (y2 - y1) * (x_max - x1) / (x2 - x1);
                x = x_max;
            }
            else if (outcode & LEFT)
            {
                y = y1 + (y2 - y1) * (x_min - x1) / (x2 - x1);
                x = x_min;
            }

            if (outcode == code1)
            {
                x1 = x;
                y1 = y;
                code1 = computeCode(x1, y1, x_min, y_min, x_max, y_max);
            }
            else
            {
                x2 = x;
                y2 = y;
                code2 = computeCode(x2, y2, x_min, y_min, x_max, y_max);
            }
        }
    }
    if (accept)
    {
        line(int(x1), int(y1), int(x2), int(y2));
    }
}

int main()
{
    double x_min, y_min, x_max, y_max;
    double x1, y1, x2, y2;

    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);

    cout << "Enter the coordinates of the clipping window (x_min, y_min, x_max, y_max): ";
    cin >> x_min >> y_min >> x_max >> y_max;

    rectangle(x_min, y_min, x_max, y_max);

    cout << "Enter the coordinates of the line (x1, y1, x2, y2): ";
    cin >> x1 >> y1 >> x2 >> y2;

    setcolor(RED);
    line(int(x1), int(y1), int(x2), int(y2));

    setcolor(WHITE);
    cohenSutherlandClip(x1, y1, x2, y2, x_min, y_min, x_max, y_max);

    getch();
    closegraph();

    return 0;
}

// mplca

#include <iostream>
#include <graphics.h>
#include <math.h>

#define LEFT 1
#define RIGHT 4
#define TOP 8
#define BOTTOM 2

using namespace std;

int xmin, xmax, ymin, ymax;

int getRegionCode(float x, float y)
{
    int code = 0;
    if (x < xmin)
        code |= LEFT;
    if (x > xmax)
        code |= RIGHT;
    if (y < ymin)
        code |= TOP;
    if (y > ymax)
        code |= BOTTOM;

    return code;
}

void drawClippedLine(float x1, float y1, float x2, float y2)
{
    line(x1, y1, x2, y2);
}

void midpointsubdivision(float x1, float y1, float x2, float y2)
{
    int code1 = getRegionCode(x1, y1);
    int code2 = getRegionCode(x2, y2);

    if (code1 == 0 && code2 == 0)
    {
        drawClippedLine(x1, y1, x2, y2);
        return;
    }
    else if ((code1 & code2) != 0)
    {
        return;
    }

    float xm = (x1 + x2) / 2;
    float ym = (y1 + y2) / 2;
    int codem = getRegionCode(xm, ym);

    if (codem != 0)
    {
        if ((code1 & codem) != 0)
        {
            midpointsubdivision(x1, y1, xm, ym);
        }
        else if ((code2 & codem) != 0)
        {
            midpointsubdivision(xm, ym, x2, y2);
        }
    }
    else
    {
        drawClippedLine(x1, y1, xm, ym);
        drawClippedLine(xm, ym, x2, y2);
    }
}

int main()
{
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);

    cout << "Enter XMIN: ";
    cin >> xmin;
    cout << "Enter YMIN: ";
    cin >> ymin;
    cout << "Enter XMAX: ";
    cin >> xmax;
    cout << "Enter YMAX: ";
    cin >> ymax;

    rectangle(xmin, ymin, xmax, ymax);

    float x1, y1, x2, y2;
    cout << "Enter POINTS A AND B: (X1, Y1) THEN (X2, Y2) ";
    cin >> x1 >> y1;
    cin >> x2 >> y2;

    midpointsubdivision(x1, y1, x2, y2);

    delay(5000);
    closegraph();
    return 0;
}

// lblca

#include <iostream>
#include <graphics.h>
using namespace std;

void liangBarskyLineClip(double x1, double y1, double x2, double y2, double xmin, double ymin, double xmax, double ymax)
{
    double t1 = 0, t2 = 1;

    double dx = x2 - x1;
    double dy = y2 - y1;

    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x1 - xmin, xmax - x1, y1 - ymin, ymax - y1};

    for (int i = 0; i < 4; i++)
    {
        if (p[i] == 0)
        {
            if (q[i] < 0)
            {
                cout << "Line is parallel to clipping window and outside.\n";
                return;
            }
        }
        else
        {
            double t = q[i] / p[i];
            if (p[i] < 0 && t > t1)
                t1 = t;
            else if (p[i] > 0 && t < t2)
                t2 = t;
        }
    }

    setcolor(GREEN);

    if (t1 < t2)
    {
        double x_new1 = x1 + t1 * dx;
        double y_new1 = y1 + t1 * dy;
        double x_new2 = x1 + t2 * dx;
        double y_new2 = y1 + t2 * dy;

        line(x_new1, y_new1, x_new2, y_new2);
    }
    else
    {
        cout << "Line is completely outside the clipping window.\n";
        setcolor(RED);
    }
}

void clipPolygon(double vertices[], int n, double xmin, double ymin, double xmax, double ymax)
{
    for (int i = 0; i < 2 * n; i += 2)
    {
        int next = (i + 2) % (2 * n);

        liangBarskyLineClip(vertices[i], vertices[i + 1], vertices[next], vertices[next + 1], xmin, ymin, xmax, ymax);
    }
}

int main()
{
    double xmin = 50, ymin = 50, xmax = 300, ymax = 200;

    double vertices[12] = {150, 150, 200, 100, 250, 150, 250, 200, 200, 250, 150, 200};

    initwindow(400, 300, "Liang-Barsky Polygon Clipping Algorithm");

    rectangle(xmin, ymin, xmax, ymax);

    int verticesInt[12];
    for (int i = 0; i < 12; i++)
        verticesInt[i] = static_cast<int>(vertices[i]);

    drawpoly(6, verticesInt);

    clipPolygon(vertices, 6, xmin, ymin, xmax, ymax);

    delay(50000);
    // closegraph();

    return 0;
}

// 2d trans
#include <bits/stdc++.h>
using namespace std;

struct Point
{
    float x, y;

    Point(float x, float y) : x(x), y(y) {}
};

vector<Point> applyTransformation(const std::vector<Point> &points, const std::vector<std::vector<float>> &transformation)
{
    vector<Point> transformedPoints;
    for (const auto &point : points)
    {
        float x = point.x * transformation[0][0] + point.y * transformation[0][1] + transformation[0][2];
        float y = point.x * transformation[1][0] + point.y * transformation[1][1] + transformation[1][2];
        transformedPoints.emplace_back(x, y);
    }
    return transformedPoints;
}

int main()
{
    int numSides;
    cout << "How many sides your polygon has?: ";
    cin >> numSides;

    vector<Point> polygonPoints;
    cout << "Enter the coordinates of the vertices:\n";
    for (int i = 0; i < numSides; ++i)
    {
        float x, y;
        std::cout << "Vertex " << (i + 1) << ": ";
        std::cin >> x >> y;
        polygonPoints.emplace_back(x, y);
    }

    vector<std::vector<float>> transformationMatrix(3, std::vector<float>(3, 0.0f));

    char choice;
    do
    {
        cout << "Which transformation you want? (T: Translation, R: Rotation, S: Scaling): ";
        cin >> choice;

        switch (choice)
        {
        case 'T':
        {
            float tx, ty;
            cout << "Enter translation vector (tx ty): ";
            cin >> tx >> ty;
            transformationMatrix = {
                {1, 0, tx},
                {0, 1, ty},
                {0, 0, 1}};
            break;
        }
        case 'R':
        {
            float angle;
            cout << "Enter rotation angle (in degrees): ";
            cin >> angle;
            angle = angle * M_PI / 180.0f;
            float cosAngle = cos(angle);
            float sinAngle = sin(angle);
            transformationMatrix = {
                {static_cast<float>(cosAngle), static_cast<float>(-sinAngle), 0},
                {static_cast<float>(sinAngle), static_cast<float>(cosAngle), 0},
                {0, 0, 1}};
            break;
        }
        case 'S':
        {
            float sx, sy;
            cout << "Enter scaling factors (Sx Sy): ";
            cin >> sx >> sy;
            transformationMatrix = {
                {sx, 0, 0},
                {0, sy, 0},
                {0, 0, 1}};
            break;
        }
        default:
            cout << "Invalid input!\n";
        }

        polygonPoints = applyTransformation(polygonPoints, transformationMatrix);

        cout << "Transformed Polygon Points:\n";
        for (const auto &point : polygonPoints)
        {
            cout << "(" << point.x << ", " << point.y << ")\n";
        }

        cout << "Do you wish to continue transformation? (Y/N): ";
        cin >> choice;
    } while (choice == 'Y' || choice == 'y');

    return 0;
}

// 3d
#include <bits/stdc++.h>
using namespace std;

struct Point
{
    float x, y, z;

    Point(float x, float y, float z) : x(x), y(y), z(z) {}
};

vector<Point> applyTransformation(const std::vector<Point> &points, const std::vector<std::vector<float>> &transformation)
{
    vector<Point> transformedPoints;
    for (const auto &point : points)
    {
        float x = point.x * transformation[0][0] + point.y * transformation[0][1] + point.z * transformation[0][2] + transformation[0][3];
        float y = point.x * transformation[1][0] + point.y * transformation[1][1] + point.z * transformation[1][2] + transformation[1][3];
        float z = point.x * transformation[2][0] + point.y * transformation[2][1] + point.z * transformation[2][2] + transformation[2][3];
        transformedPoints.emplace_back(x, y, z);
    }
    return transformedPoints;
}

vector<vector<float>> rotationMatrix(char axis, float angle)
{
    angle = angle * M_PI / 180.0f;
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);
    vector<vector<float>> rotation;
    if (axis == 'x' || axis == 'X')
    {
        rotation = {
            {1, 0, 0, 0},
            {0, cosAngle, -sinAngle, 0},
            {0, sinAngle, cosAngle, 0},
            {0, 0, 0, 1}};
    }
    else if (axis == 'y' || axis == 'Y')
    {
        rotation = {
            {cosAngle, 0, sinAngle, 0},
            {0, 1, 0, 0},
            {-sinAngle, 0, cosAngle, 0},
            {0, 0, 0, 1}};
    }
    else if (axis == 'z' || axis == 'Z')
    {
        rotation = {
            {cosAngle, -sinAngle, 0, 0},
            {sinAngle, cosAngle, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}};
    }
    else
    {
        cout << "Invalid axis!\n";
    }
    return rotation;
}

int main()
{
    int numVertices;
    cout << "How many vertices your 3D object has?: ";
    cin >> numVertices;

    vector<Point> objectPoints;
    cout << "Enter the coordinates of the vertices:\n";
    for (int i = 0; i < numVertices; ++i)
    {
        float x, y, z;
        std::cout << "Vertex " << (i + 1) << ": ";
        std::cin >> x >> y >> z;
        objectPoints.emplace_back(x, y, z);
    }

    vector<vector<float>> transformationMatrix(4, vector<float>(4, 0.0f));

    char choice;
    do
    {
        cout << "Which transformation you want? (T: Translation, R: Rotation, S: Scaling): ";
        cin >> choice;

        switch (choice)
        {
        case 'T':
        {
            float tx, ty, tz;
            cout << "Enter translation vector (tx ty tz): ";
            cin >> tx >> ty >> tz;
            transformationMatrix = {
                {1, 0, 0, tx},
                {0, 1, 0, ty},
                {0, 0, 1, tz},
                {0, 0, 0, 1}};
            break;
        }
        case 'R':
        {
            char axis;
            float angle;
            cout << "Enter rotation axis (X, Y, Z): ";
            cin >> axis;
            cout << "Enter rotation angle (in degrees): ";
            cin >> angle;
            transformationMatrix = rotationMatrix(axis, angle);
            break;
        }
        case 'S':
        {
            float sx, sy, sz;
            cout << "Enter scaling factors (Sx Sy Sz): ";
            cin >> sx >> sy >> sz;
            transformationMatrix = {
                {sx, 0, 0, 0},
                {0, sy, 0, 0},
                {0, 0, sz, 0},
                {0, 0, 0, 1}};
            break;
        }
        default:
            cout << "Invalid input!\n";
        }

        objectPoints = applyTransformation(objectPoints, transformationMatrix);

        cout << "Transformed Object Points:\n";
        for (const auto &point : objectPoints)
        {
            cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
        }

        cout << "Do you wish to continue transformation? (Y/N): ";
        cin >> choice;
    } while (choice == 'Y' || choice == 'y');

    return 0;
}

// 2d images
#include <GL/glut.h>
#include <cmath>

void drawSquare()
{
    glBegin(GL_QUADS);
    glVertex2f(-0.3, 0.3);
    glVertex2f(0.3, 0.3);
    glVertex2f(0.3, -0.3);
    glVertex2f(-0.3, -0.3);
    glEnd();
}

void drawCircle(float radius, int segments)
{
    glBegin(GL_TRIANGLE_FAN);
    for (int i = 0; i <= segments; ++i)
    {
        float theta = 2.0f * 3.1415926f * float(i) / float(segments);
        float x = radius * cosf(theta);
        float y = radius * sinf(theta);
        glVertex2f(x, y);
    }
    glEnd();
}

void drawTriangle()
{
    glBegin(GL_TRIANGLES);
    glVertex2f(0.0, 0.3);
    glVertex2f(-0.3, -0.3);
    glVertex2f(0.3, -0.3);
    glEnd();
}

void drawPentagon()
{
    glBegin(GL_POLYGON);
    for (int i = 0; i < 5; ++i)
    {
        float angle = 2 * 3.1415926f / 5 * i;
        glVertex2f(cosf(angle) * 0.3, sinf(angle) * 0.3);
    }
    glEnd();
}

void drawHexagon()
{
    glBegin(GL_POLYGON);
    for (int i = 0; i < 6; ++i)
    {
        float angle = 2 * 3.1415926f / 6 * i;
        glVertex2f(cosf(angle) * 0.3, sinf(angle) * 0.3);
    }
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw square
    glColor3f(1.0, 0.0, 0.0); // Red
    drawSquare();

    // Draw circle
    glColor3f(0.0, 1.0, 0.0); // Green
    glPushMatrix();
    glTranslatef(-0.7, 0.0, 0.0);
    drawCircle(0.3, 30);
    glPopMatrix();

    // Draw triangle
    glColor3f(0.0, 0.0, 1.0); // Blue
    glPushMatrix();
    glTranslatef(0.7, 0.0, 0.0);
    drawTriangle();
    glPopMatrix();

    // Draw pentagon
    glColor3f(1.0, 1.0, 0.0); // Yellow
    glPushMatrix();
    glTranslatef(0.0, -0.7, 0.0);
    drawPentagon();
    glPopMatrix();

    // Draw hexagon
    glColor3f(1.0, 0.0, 1.0); // Magenta
    glPushMatrix();
    glTranslatef(0.0, 0.7, 0.0);
    drawHexagon();
    glPopMatrix();

    glFlush();
}

void init()
{
    glClearColor(0.0, 0.0, 0.0, 1.0); // Set clear color to black
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1.0, 1.0, -1.0, 1.0); // Set viewing area dimensions
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(500, 500);                   // Set window size
    glutInitWindowPosition(100, 100);               // Set window position
    glutCreateWindow("Different Shapes in OpenGL"); // Create a window with the given title
    init();                                         // Initialize OpenGL
    glutDisplayFunc(display);                       // Register display callback function
    glutMainLoop();                                 // Enter the main event loop
    return 0;
}

// 3d images

// cuboid
#include <GL/glut.h>

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    gluLookAt(3, 3, 3, 0, 0, 0, 1, 0, -1); // eye, center, up vector

    // eye = camera
    // center = where the camera is focussed
    // up = which dir is up for the camera

    glBegin(GL_QUADS);

    // Front
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);

    // Back face
    glColor3f(0.0, 1.0, 0.0); // Green
    glVertex3f(-1.0, -1.0, -1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glVertex3f(-1.0, 1.0, -1.0);

    // Top face
    glColor3f(0.0, 0.0, 1.0); // Blue
    glVertex3f(-1.0, 1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glVertex3f(-1.0, 1.0, -1.0);

    // Bottom face
    glColor3f(1.0, 1.0, 0.0); // Yellow
    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, -1.0);

    // Right face
    glColor3f(1.0, 0.0, 1.0); // Magenta
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glVertex3f(1.0, 1.0, 1.0);

    // Left face
    glColor3f(0.0, 1.0, 1.0); // Cyan
    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glVertex3f(-1.0, 1.0, 1.0);

    glEnd();

    glutSwapBuffers();
}

void init()
{
    glEnable(GL_DEPTH_TEST); // Enable depth testing for 3D rendering
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0); // Set up a perspective projection
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("3D Cube Example");
    glutDisplayFunc(display);
    init();
    glutMainLoop();
    return 0;
}

// cylinder
#include <GL/glut.h>
#include <cmath>

const float PI = 3.14159265358979323846;

void drawCylinder(float radius, float height, int numSegments)
{
    glBegin(GL_QUADS);
    for (int i = 0; i < numSegments; ++i)
    {
        float theta1 = (2.0f * PI * i) / numSegments;
        float theta2 = (2.0f * PI * (i + 1)) / numSegments;

        // Top surface
        glVertex3f(radius * cos(theta1), height / 2.0f, radius * sin(theta1));
        glVertex3f(radius * cos(theta1), height / 2.0f, radius * sin(theta1));
        glVertex3f(radius * cos(theta2), height / 2.0f, radius * sin(theta2));
        glVertex3f(radius * cos(theta2), height / 2.0f, radius * sin(theta2));

        // Bottom surface
        glVertex3f(radius * cos(theta1), -height / 2.0f, radius * sin(theta1));
        glVertex3f(radius * cos(theta2), -height / 2.0f, radius * sin(theta2));
        glVertex3f(radius * cos(theta2), -height / 2.0f, radius * sin(theta2));
        glVertex3f(radius * cos(theta1), -height / 2.0f, radius * sin(theta1));

        // Side surface
        glVertex3f(radius * cos(theta1), height / 2.0f, radius * sin(theta1));
        glVertex3f(radius * cos(theta2), height / 2.0f, radius * sin(theta2));
        glVertex3f(radius * cos(theta2), -height / 2.0f, radius * sin(theta2));
        glVertex3f(radius * cos(theta1), -height / 2.0f, radius * sin(theta1));
    }
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    gluLookAt(3, 3, 3, 0, 0, 0, 0, 1, 0); // Eye position, look-at position, up vector

    glColor3f(0.5, 0.5, 0.5); // Gray color for the cylinder

    drawCylinder(1.0, 2.0, 50); // Radius, Height, Number of Segments

    glutSwapBuffers();
}

void init()
{
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("3D Cylinder Example");
    glutDisplayFunc(display);
    init();
    glutMainLoop();
    return 0;
}
// pyramid
#include <GL/glut.h>

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    gluLookAt(3, 3, 3, 0, 0, 0, 1, 0, 1);

    glBegin(GL_TRIANGLES);

    // Front face
    glColor3f(1.0, 0.0, 0.0); // Red
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);

    // Right face
    glColor3f(0.0, 1.0, 0.0); // Green
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);

    // Back face
    glColor3f(0.0, 0.0, 1.0); // Blue
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, -1.0);

    // Left face
    glColor3f(1.0, 1.0, 0.0); // Yellow
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, 1.0);

    glEnd();

    glBegin(GL_QUADS);

    // Bottom face
    glColor3f(1.0, 0.0, 1.0); // Magenta
    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, -1.0);

    glEnd();

    glutSwapBuffers();
}

void init()
{
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("3D Pyramid Example");
    glutDisplayFunc(display);
    init();
    glutMainLoop();
    return 0;
}

// sphere
#include <GL/glut.h>
#include <cmath>

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity(); // Reset the modelview matrix

    // Set up the camera
    gluLookAt(3, 3, 3, 0, 0, 0, 0, 1, 0); // eye, center, up vector

    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat lightPosition[] = {1.0, 1.0, 1.0, 0.0}; // Directional light from the top-right
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    GLfloat sphereColor[] = {0.0, 1.0, 0.0, 1.0}; // Green color for the sphere
    glMaterialfv(GL_FRONT, GL_DIFFUSE, sphereColor);

    const int slices = 30;    // Number of longitude divisions
    const int stacks = 30;    // Number of latitude divisions
    const float radius = 1.0; // Radius of the sphere

    for (int i = 0; i < slices; ++i)
    {
        float theta1 = i * (2.0 * M_PI) / slices;
        float theta2 = (i + 1) * (2.0 * M_PI) / slices;
        glBegin(GL_TRIANGLE_STRIP);
        for (int j = 0; j <= stacks; ++j)
        {
            float phi = j * M_PI / stacks;
            float x1 = radius * cos(theta1) * sin(phi);
            float y1 = radius * sin(theta1) * sin(phi);
            float z1 = radius * cos(phi);
            float x2 = radius * cos(theta2) * sin(phi);
            float y2 = radius * sin(theta2) * sin(phi);
            float z2 = radius * cos(phi);
            glNormal3f(x1, y1, z1);
            glVertex3f(x1, y1, z1);
            glNormal3f(x2, y2, z2);
            glVertex3f(x2, y2, z2);
        }
        glEnd();
    }

    glDisable(GL_LIGHTING);

    glutSwapBuffers();
}

void init()
{
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("3D Sphere Example");
    glutDisplayFunc(display);
    init();
    glutMainLoop();
    return 0;
}
