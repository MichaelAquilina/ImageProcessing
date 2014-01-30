using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using RoadExtraction.Geometry;
using RoadExtraction.Objects;
using RoadExtraction.Imagery;

namespace RoadExtraction.ImageProcessing
{
    //specify what is expected out of a distance function
    public delegate double DistanceFunction ( byte[] vector1, byte[] vector2 );

    /// <summary>
    /// Generalized Image Analysis class with a variety of image processing functions
    /// </summary>
    public class ImageAnalysis
    {
        #region Private/Internal Methods

        private static double Intensity(byte[] vector)
        {
            //blue green red, alpha is IGNORED
            return 0.114 * vector[0] + 0.587 * vector[1] + 0.299 * vector[2];
        }

        public static double IntensityDistance(byte[] vector1, byte[] vector2)
        {
            if (vector1.Length < 3 || vector2.Length < 3)
                throw new ArgumentException("Intensity Calculations require a minimum of an RGB vector");

            double Intensity1 = Intensity(vector1);
            double Intensity2 = Intensity(vector2);

            return Math.Abs(Intensity1 - Intensity2);
        }

        public static double EuclidianDistance(byte[] vector1, byte[] vector2)
        {
            if (vector1.Length != vector2.Length)
                throw new ArgumentException("Must be of the same length");

            double total = 0.0;
            for (int i = 0; i < vector1.Length; i++)
                total += Math.Pow(vector1[i] - vector2[i], 2);

            return Math.Sqrt(total);
        }

        private static void ConnectCanny(DistanceMap EdgeMap, BinaryImage Output, Pixel Pixel, int Lower)
        {
            Output[Pixel] = BinaryImage.On;

            for (int k = 0; k < Pixel.Neighbors.Length; k++)
            {
                Pixel n = Pixel + Pixel.Neighbors[k];

                if (n.X < 0 || n.Y < 0 || n.X >= EdgeMap.Width || n.Y >= EdgeMap.Height)
                    continue;

                if (EdgeMap[n] >= Lower && Output[n] != BinaryImage.On)
                    ConnectCanny(EdgeMap, Output, n, Lower);
            }
        }

        private static DistanceMap Canny(RasterImage Image)
        {
            Image = ToGrayScale(Image);
            Image = GaussianBlurring(Image);

            //placeholders for values of Mx and My for the initial scan
            DistanceMap MxMap = new DistanceMap(Image.Width, Image.Height);
            DistanceMap MyMap = new DistanceMap(Image.Width, Image.Height);
            DistanceMap EdgeMap = new DistanceMap(Image.Width, Image.Height);

            Kernel Kx = new Kernel(3, new int[]{ -1,  0,  1,
                                                 -2,  0,  2,
                                                 -1,  0,  1 });

            Kernel Ky = new Kernel(3, new int[]{  1,  2,  1,
                                                  0,  0,  0,
                                                 -1, -2, -1 });

            //use the sobel operators to detect edge magnitude
            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    int Mx = 0;
                    int My = 0;

                    for (int x = Kx.Start; x < Kx.End; x++)
                    {
                        for (int y = Ky.Start; y < Ky.End; y++)
                        {
                            if (i + x < 0 || j + y < 0 || i + x >= Image.Width || j + y >= Image.Height)
                                continue;

                            Mx += Kx[x - Kx.Start, y - Kx.Start] * Image[i + x, j + y][0];
                            My += Ky[x - Ky.Start, y - Ky.Start] * Image[i + x, j + y][0];
                        }
                    }

                    MxMap[i, j] = Mx;
                    MyMap[i, j] = My;

                    EdgeMap[i, j] = (int)Math.Sqrt(Mx * Mx + My * My);
                }
            }

            DistanceMap NewEdgeMap = new DistanceMap(Image.Width, Image.Height);

            //non-maximal suprression
            for (int i = 1; i < Image.Width - 1; i++)
            {
                for (int j = 1; j < Image.Height - 1; j++)
                {
                    //calculate the edge direction using Mx and My
                    double Angle = (Math.Atan2(MyMap[i, j], MxMap[i, j]) / Math.PI) * 180;
                    Pixel M;

                    //discretise the angles
                    if (((Angle < 22.5) && (Angle > -22.5)) || (Angle > 157.5) || (Angle < -157.5))
                        M = new Pixel(1, 0);    //0 degrees
                    else if (((Angle > 22.5) && (Angle < 67.5)) || ((Angle < -112.5) && (Angle > -157.5)))
                        M = new Pixel(1, -1);   //45 degrees
                    else if (((Angle > 67.5) && (Angle < 112.5)) || ((Angle < -67.5) && (Angle > -112.5)))
                        M = new Pixel(0, -1);   //90 degrees
                    else if (((Angle > 112.5) && (Angle < 157.5)) || ((Angle < -22.5) && (Angle > -67.5)))
                        M = new Pixel(-1, -1);  //135 degrees
                    else M = new Pixel(0, 0);   //should never reach this case

                    Pixel M1 = new Pixel(M.X + i, M.Y + j);
                    Pixel M2 = new Pixel(i - M.X, j - M.Y);

                    if (EdgeMap[i, j] > EdgeMap[M1] && EdgeMap[i, j] > EdgeMap[M2])
                        NewEdgeMap[i, j] = EdgeMap[i, j];
                    else
                        NewEdgeMap[i, j] = 0;
                }
            }

            return NewEdgeMap;
        }

        #endregion

        //organised as even,odd,even,odd in order to allow calculation of 3-4 distance transform
        private static Pixel[] BeforePixels = new Pixel[] { new Pixel(-1, -1), new Pixel(0, -1), new Pixel(-1, 1), new Pixel(-1, 0) };
        private static Pixel[] AfterPixels = new Pixel[] { new Pixel(+1, -1), new Pixel(1, 0), new Pixel(1, 1), new Pixel(0, 1) };

        //new operation found in a paper which attempts to clean an image based on neighborhood counting
        public static BinaryImage Clean(BinaryImage Image, int KernelSize)
        {
            if (KernelSize % 2 == 0)
                throw new ArgumentException("Kernel Window Size must be an odd number");

            int XStart = Convert.ToInt32(-1 * Math.Floor((double)KernelSize / 2));
            int XEnd = Convert.ToInt32(Math.Ceiling((double)KernelSize / 2));

            int YStart = Convert.ToInt32(-1 * Math.Floor((double)KernelSize / 2));
            int YEnd = Convert.ToInt32(Math.Ceiling((double)KernelSize / 2));

            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    int White = 0;
                    int Black = 0;

                    for (int x = XStart; x < XEnd; x++)
                    {
                        for (int y = YStart; y < YEnd; y++)
                        {
                            if (i + x < 0 || j + y < 0 || i + x >= Image.Width || j + y >= Image.Height)
                                continue;

                            if (Image[i + x, j + y] == BinaryImage.On)
                                White++;
                            else
                                Black++;
                        }
                    }

                    if (White >= Black)
                        Output[i, j] = BinaryImage.On;
                    else
                        Output[i, j] = BinaryImage.Off;
                }
            }

            return Output;
        }

        public static DistanceMap DistanceTransform(BinaryImage Image)
        {
            return DistanceTransform(Image,BinaryImage.On);
        }

        //applies and returns an image containing the data from a 3-4 distance transform on the binary image
        public static DistanceMap DistanceTransform(BinaryImage Image, byte Search)
        {
            DistanceMap F1Output = new DistanceMap(Image.Width, Image.Height);
            DistanceMap F2Output = new DistanceMap(Image.Width, Image.Height);

            //first pass through the image
            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    if (i == 0 && j == 0)
                        continue;

                    if (Image[i, j] != Search )
                        F1Output[i, j] = 0;
                    else
                    {
                        int Min = Int32.MaxValue;

                        //go through all the pixels in the before pixel set
                        for (int index = 0; index < BeforePixels.Length; index++)
                        {
                            Pixel P = BeforePixels[index];
                            Pixel pixel = new Pixel(P.X + i, P.Y + j);
                            int value = (index % 2 == 0) ? 4 : 3;

                            if (pixel.X < 0 || pixel.Y < 0 || pixel.X >= Image.Width || pixel.Y >= Image.Height)
                                continue;

                            if (F1Output[pixel.X, pixel.Y] + value < Min)
                                Min = F1Output[pixel.X, pixel.Y] + value;
                        }

                        F1Output[i, j] = Min;
                    }
                }
            }

            //second pass through the image
            for (int i = Image.Width - 1; i >= 0; i--)
            {
                for (int j = Image.Height - 1; j >= 0; j--)
                {
                    if (i == Image.Width - 1 && j == Image.Height - 1)
                        continue;

                    int Min = F1Output[i, j];

                    //go through all the pixels in the before pixel set
                    for (int index = 0; index < AfterPixels.Length; index++)
                    {
                        Pixel P = AfterPixels[index];
                        Pixel pixel = new Pixel(P.X + i, P.Y + j);
                        int value = (index % 2 == 0) ? 4 : 3;

                        if (pixel.X < 0 || pixel.Y < 0 || pixel.X >= Image.Width || pixel.Y >= Image.Height)
                            continue;

                        if (F2Output[pixel.X, pixel.Y] + value < Min)
                            Min = F2Output[pixel.X, pixel.Y] + value;
                    }

                    F2Output[i, j] = Min;
                }
            }

            return F2Output;
        }

        public static BinaryImage Invert(BinaryImage Image)
        {
            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);
            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    Output[i, j] = (Image[i, j] == BinaryImage.On) ? BinaryImage.Off : BinaryImage.On;
                }
            }
            return Output;
        }

        //removes the border pixels from the given binary image
        public static BinaryImage RemoveBorder(BinaryImage Image)
        {
            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            for (int i = 1; i < Image.Width - 1; i++)
                for (int j = 1; j < Image.Height - 1; j++)
                    Output[i, j] = Image[i, j];

            return Output;
        }

        //segments an image using a flood fill algorithm with a tolerance to similarity specified in the parameters
        public static List<PixelGroup> Segment(RasterImage Image, BinaryImage EdgeMap, double tolerance, int seedDistance, DistanceFunction Distance )
        {
            List<PixelGroup> Output = new List<PixelGroup>();

            BinaryImage VisitedImage = new BinaryImage(Image.Width, Image.Height);
            Queue<Pixel> PixelQueue = new Queue<Pixel>();

            for (int i = 3; i < Image.Width-3; i += seedDistance)
            {
                for (int j = 3; j < Image.Height-3; j += seedDistance)
                {
                    //perform a flood fill with tolerence specified in parameters
                    PixelGroup Group = new PixelGroup(Image.Width, Image.Height);
                    PixelQueue.Enqueue(new Pixel(i, j));

                    do
                    {
                        Pixel pixel = PixelQueue.Dequeue();

                        if (VisitedImage[pixel] == BinaryImage.On )
                            continue;

                        if( EdgeMap!=null && EdgeMap[pixel] == BinaryImage.On )
                            continue;

                        VisitedImage[pixel] = BinaryImage.On;
                        Group.AddPixel(pixel);

                        //for its neighborhood, if a pixel is similiar within some level of tolerance to this pixel then add it to the queue
                        for (int x = -1; x < 2; x++)
                        {
                            for (int y = -1; y < 2; y++)
                            {
                                if (pixel.X + x < 0 || pixel.Y + y < 0 || pixel.X + x >= Image.Width || pixel.Y + y >= Image.Height)
                                    continue;

                                if (x == 0 && y == 0)
                                    continue;

                                if (Distance(Image[pixel.X + x, pixel.Y + y], Image[pixel.X, pixel.Y]) < tolerance)
                                    PixelQueue.Enqueue(new Pixel(pixel.X + x, pixel.Y + y));
                            }
                        }

                    } while (PixelQueue.Count > 0);
                        
                    Output.Add(Group);
                }
            }

            return Output;
        }

        //performs gaussian blurring using appropriate convolution masks
        public static RasterImage GaussianBlurring(RasterImage Image)
        {
            Kernel Kernel = new Kernel(5, new int[] { 2 , 4 , 5 , 4 , 2 ,
                                                      4 , 9 , 12, 9 , 4 ,
                                                      5 , 12, 15, 12, 5 ,
                                                      4 , 9 , 12, 9 , 4 ,
                                                      2 , 4 , 5 , 4 , 2   });
            Kernel.Denominator = 159;

            Image.ApplyKernel(Kernel);
            return Image;
        }

        //changes an input raster image into a grayscale format
        public static BinaryImage ToGrayScale(RasterImage Image)
        {
            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            if (Image.BytesPerPixel == 1)
            {
                Output.Pixels = Image.Pixels;
                return Output;
            }

            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    int value = 0;

                    //important to ignore the last value (Alpha)
                    for (int k = 0; k < Image.BytesPerPixel && k < 3; k++)
                        value += Image[i, j][k];

                    Output[i, j] = Convert.ToByte(value / 3);
                }
            }

            return Output;
        }

        //Changes a given list segments into a colored representation
        public static ArgbImage SegmentToImage(List<PixelGroup> Segments)
        {
            int[] Colors = new int[] { ArgbColors.Red, ArgbColors.Green, ArgbColors.Yellow, ArgbColors.Blue, ArgbColors.Orange, ArgbColors.SteelBlue, ArgbColors.Purple, ArgbColors.OrangeRed };

            ArgbImage Image = new ArgbImage(Segments[0].Width, Segments[0].Height);

            for (int i = 0; i < Image.Width; i++)
                for (int j = 0; j < Image.Height; j++)
                    Image[i, j] = ArgbImage.ArgbToInt(0, 0, 0, 0);

            int index = 0;

            foreach (PixelGroup Group in Segments)
            {
                int color = Colors[(index++)%Colors.Length];

                foreach (Pixel pixel in Group.Pixels)
                    Image[pixel.X, pixel.Y] = color;
            }

            return Image;
        }

        //performs sobel edge detection
        public static BinaryImage SobelEdgeDetection(RasterImage Image)
        {
            DistanceMap EdgeMap = new DistanceMap(Image.Width, Image.Height);

            Kernel Kx = new Kernel(3, new int[]{ -1,  0,  1,
                                                 -2,  0,  2,
                                                 -1,  0,  1 });

            Kernel Ky = new Kernel(3, new int[]{  1,  2,  1,
                                                  0,  0,  0,
                                                 -1, -2, -1 });

            //use the sobel operators to detect edge magnitude
            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    int Mx = 0;
                    int My = 0;

                    for (int x = Kx.Start; x < Kx.End; x++)
                    {
                        for (int y = Ky.Start; y < Ky.End; y++)
                        {
                            if (i + x < 0 || j + y < 0 || i + x >= Image.Width || j + y >= Image.Height)
                                continue;

                            Mx += Kx[x - Kx.Start, y - Kx.Start] * Image[i + x, j + y][0];
                            My += Ky[x - Ky.Start, y - Ky.Start] * Image[i + x, j + y][0];
                        }
                    }

                    EdgeMap[i, j] = (int)Math.Sqrt(Mx * Mx + My * My);
                }
            }

            return EdgeMap.ToBinaryImage(1);
        }

        //performs canny edge detection
        public static BinaryImage CannyEdgeDetection(RasterImage Image, int Upper, int Lower)
        {
            DistanceMap NewEdgeMap = Canny(Image);
            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            //hyesterisis thresholding
            for (int i = 0; i < Image.Width; i++)
                for (int j = 0; j < Image.Height; j++)
                    if (NewEdgeMap[i, j] >= Upper)
                        ConnectCanny(NewEdgeMap, Output, new Pixel(i, j), Lower);

            return Output;
        }
    }
}
