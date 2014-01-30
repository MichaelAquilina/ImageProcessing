using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using RoadExtraction.Objects;
using RoadExtraction.Imagery;

namespace RoadExtraction.ImageProcessing
{
    //types of connectivity to use in flood fill algorithms
    public enum ConnectivityType { FourWayConnectivity, EightWayConnectivity };

    public class Grouping
    {
        //returns a group of pixels found in the binary image with the given search criteria, minimum size to be considered a group and connectivity type to search with
        public static List<PixelGroup> GroupPixels(BinaryImage Image, byte search, int minSize, ConnectivityType ConnectivityType, bool ignoreBorder )
        {
            Queue<Pixel> PixelQueue = new Queue<Pixel>();

            List<PixelGroup> GroupList = new List<PixelGroup>();
            BinaryImage TakenImage = new BinaryImage(Image.Width, Image.Height);

            for (int i = 0; i < Image.Width; i++)
            {
                for (int j = 0; j < Image.Height; j++)
                {
                    //if a search pixel is found then perform a flood fill to determine its grouping
                    if (Image[i, j] == search && TakenImage[i, j] != BinaryImage.On)
                    {
                        PixelGroup Group = new PixelGroup(Image.Width, Image.Height);
                        PixelQueue.Enqueue(new Pixel(i, j));
                        bool cancel = false;

                        do
                        {
                            Pixel pixel = PixelQueue.Dequeue();

                            if (pixel.X < 0 || pixel.Y < 0 || pixel.X >= Image.Width || pixel.Y >= Image.Height)
                            {
                                cancel = true && ignoreBorder;
                                continue;
                            }

                            if (Image[pixel.X, pixel.Y] != search)
                                continue;

                            if (TakenImage[pixel.X, pixel.Y] == BinaryImage.On)
                                continue;

                            TakenImage[pixel.X, pixel.Y] = BinaryImage.On;
                            Group.AddPixel(pixel);

                            //enqueue neighbouring pixels based on the connecvtivity type specified
                            if (ConnectivityType == ConnectivityType.EightWayConnectivity)
                            {
                                PixelQueue.Enqueue(new Pixel(pixel.X + 1, pixel.Y + 1));
                                PixelQueue.Enqueue(new Pixel(pixel.X + 1, pixel.Y - 1));
                                PixelQueue.Enqueue(new Pixel(pixel.X - 1, pixel.Y - 1));
                                PixelQueue.Enqueue(new Pixel(pixel.X - 1, pixel.Y + 1));
                            }

                            PixelQueue.Enqueue(new Pixel(pixel.X + 1, pixel.Y));
                            PixelQueue.Enqueue(new Pixel(pixel.X, pixel.Y + 1));
                            PixelQueue.Enqueue(new Pixel(pixel.X, pixel.Y - 1));
                            PixelQueue.Enqueue(new Pixel(pixel.X - 1, pixel.Y));


                        } while (PixelQueue.Count > 0);

                        //only add the group if it passes the minimum size requried
                        if (Group.Pixels.Count >= minSize && !cancel)
                            GroupList.Add(Group);
                    }
                }
            }
            return GroupList;
        }

        //returns groups of pixels found in the binary image with the given search criteria
        public static List<PixelGroup> GroupPixels(BinaryImage Image, byte search)
        {
            return GroupPixels(Image, search, 0, ConnectivityType.EightWayConnectivity, false);
        }

        public static BinaryImage GroupClosing(BinaryImage Image, int MinSize)
        {
            return GroupPixelsTest(Image, BinaryImage.Off, BinaryImage.Off, BinaryImage.On, MinSize, ConnectivityType.EightWayConnectivity);
        }

        public static BinaryImage GroupOpening(BinaryImage Image, int MinSize)
        {
            return GroupPixelsTest(Image, BinaryImage.On, BinaryImage.On, BinaryImage.Off, MinSize, ConnectivityType.EightWayConnectivity);
        }

        //group pixel test searches for groups in the given search byte and then changes the group. Based on the idea of a hit and miss transform
        public static BinaryImage GroupPixelsTest(BinaryImage Image, byte search, byte hit, byte miss, int K, ConnectivityType ConnectivityType)
        {
            List<PixelGroup> GroupList = GroupPixels(Image, search, 1, ConnectivityType, true);

            foreach (PixelGroup Group in GroupList)
            {
                byte value = (Group.Count < K) ? miss : hit;

                foreach (Pixel pixel in Group.Pixels)
                    Image[pixel.X, pixel.Y] = value;
            }

            return Image;
        }

        //simple clustering of adjacent points without a limit
        public static BinaryImage ClusterPoints(BinaryImage Image)
        {
            List<PixelGroup> GroupList = GroupPixels(Image, BinaryImage.On, 1, ConnectivityType.EightWayConnectivity, false);
            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            foreach (PixelGroup Group in GroupList)
            {
                int X = (int)(Group.MinX + Group.MaxX) / 2;
                int Y = (int)(Group.MinY + Group.MaxY) / 2;

                if (Image[X, Y] == BinaryImage.On)
                    Output[X, Y] = BinaryImage.On;
                else
                {
                    //search the sorrounding neighbourhood for good positions in the image
                    //choose the pixel in the group which is closest to the center
                    Pixel ideal = new Pixel();
                    int min = Int32.MaxValue;
                    foreach (Pixel pixel in Group.Pixels)
                    {
                        int distance = (int)Math.Sqrt(Math.Pow(pixel.X - X, 2) + Math.Pow(pixel.Y - Y, 2));
                        if (distance < min)
                        {
                            ideal = pixel;
                            min = distance;
                        }
                    }

                    Output[ideal.X, ideal.Y] = BinaryImage.On;
                }
            }

            return Output;
        }

        #region Pixel Group Operations

        public static PixelGroup GroupClosing(PixelGroup Input, int MinSize)
        {
            return GroupPixelsTest(Input, BinaryImage.Off, BinaryImage.On, MinSize, ConnectivityType.EightWayConnectivity);
        }

        //returns a group of pixels found in the binary image with the given search criteria, minimum size to be considered a group and connectivity type to search with
        public static List<PixelGroup> GroupPixels(PixelGroup Input, byte search, int minSize, ConnectivityType ConnectivityType)
        {
            Queue<Pixel> PixelQueue = new Queue<Pixel>();

            List<PixelGroup> GroupList = new List<PixelGroup>();
            BinaryImage TakenImage = new BinaryImage(Input.Width, Input.Height);

            bool Search = (search == BinaryImage.On) ? true : false;

            for (int i = Input.MinX; i <= Input.MaxX; i++)
            {
                for (int j = Input.MinY; j <= Input.MaxY; j++)
                {
                    //if a search pixel is found then perform a flood fill to determine its grouping
                    if (Input.HasPixel(i,j)==Search && TakenImage[i, j] != BinaryImage.On)
                    {
                        PixelGroup Group = new PixelGroup(Input.Width, Input.Height);
                        PixelQueue.Enqueue(new Pixel(i, j));
                        bool cancel = false;

                        do
                        {
                            Pixel pixel = PixelQueue.Dequeue();

                            if (pixel.X < Input.MinX || pixel.Y < Input.MinY || pixel.X > Input.MaxX || pixel.Y > Input.MaxY)
                            {
                                cancel = true;
                                break;
                            }

                            if (Input.HasPixel(pixel)!=Search)
                                continue;

                            if (TakenImage[pixel.X, pixel.Y] == BinaryImage.On)
                                continue;

                            TakenImage[pixel.X, pixel.Y] = BinaryImage.On;
                            Group.AddPixel(pixel);

                            //enqueue neighbouring pixels based on the connecvtivity type specified
                            if (ConnectivityType == ConnectivityType.EightWayConnectivity)
                            {
                                PixelQueue.Enqueue(new Pixel(pixel.X + 1, pixel.Y + 1));
                                PixelQueue.Enqueue(new Pixel(pixel.X + 1, pixel.Y - 1));
                                PixelQueue.Enqueue(new Pixel(pixel.X - 1, pixel.Y - 1));
                                PixelQueue.Enqueue(new Pixel(pixel.X - 1, pixel.Y + 1));
                            }

                            PixelQueue.Enqueue(new Pixel(pixel.X + 1, pixel.Y));
                            PixelQueue.Enqueue(new Pixel(pixel.X, pixel.Y + 1));
                            PixelQueue.Enqueue(new Pixel(pixel.X, pixel.Y - 1));
                            PixelQueue.Enqueue(new Pixel(pixel.X - 1, pixel.Y));


                        } while (PixelQueue.Count > 0);

                        //only add the group if it passes the minimum size requried
                        if (Group.Pixels.Count >= minSize && !cancel)
                            GroupList.Add(Group);
                    }
                }
            }
            return GroupList;
        }

        public static PixelGroup GroupPixelsTest(PixelGroup Input, byte hit, byte miss, int K, ConnectivityType ConnectivityType)
        {
            List<PixelGroup> GroupList = GroupPixels(Input, BinaryImage.Off, 1, ConnectivityType);

            foreach (PixelGroup Group in GroupList)
            {
                byte value = (Group.Count < K) ? miss : hit;

                foreach (Pixel pixel in Group.Pixels)
                    if (value == BinaryImage.On)
                        Input.AddPixel(pixel);
                    else
                        Input.RemovePixel(pixel);
            }

            return Input;
        }

        #endregion
    }
}
