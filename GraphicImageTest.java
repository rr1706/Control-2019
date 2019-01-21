import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;
import java.net.*;
import java.util.ArrayList;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;


 class GraphicImageTest {
    public static void main(String[] args) {
        JFrame f = new JFrame();
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.getContentPane().add(new GraphicImagePanel());
        f.getContentPane().add(new GraphicImagePanel());
        f.setSize(1024,820);
        f.setLocation(512,440);
        f.setVisible(true);
    }
     public final ArrayList<Integer> xcoords = new ArrayList<>();
     private final ArrayList<Integer> ycoords = new ArrayList<>();
     private final ArrayList<Integer> ArcX = new ArrayList<>();
     private final ArrayList<Integer> ArcY = new ArrayList<>();
}
//Click points
 class GraphicImagePanel extends JPanel implements MouseMotionListener, MouseListener {
    Image image;
    //Ellipse2D.Double ball;

    public GraphicImagePanel() {
        loadImage();
        GraphicMover mover = new GraphicMover(this);
        addMouseListener(this);
        addMouseMotionListener(this);
    }

    private final ArrayList<Integer> xcoords = new ArrayList<>();
    private final ArrayList<Integer> ycoords = new ArrayList<>();
    private final ArrayList<Integer> ArcX = new ArrayList<>();
    private final ArrayList<Integer> ArcY = new ArrayList<>();
// To restart spline at the end of the arcs
    private final ArrayList<Integer> xcoords2 = new ArrayList<>();
    private final ArrayList<Integer> ycoords2 = new ArrayList<>();


    private double[] move = {0, 0};

    double t = 0;
/*

The real length of the field is required for this to work. Once measured, the grid can be scaled to make points

Brute force code, will need to be updated in ArrayList numbers and in dragging and painting methods with each new arc added
Picture X: 160-850 pixels
Picture Y: 45-735 pixels

Scale field dimensions: 690 by 690 pixels
Half field dimensions: 823 by 823 centimeters
 */
    double[] prevPoint = {0, 0};
    double distanceDelta = 0;
    int[] prevPoint2 = {0, 0};
    double[] currentPoint = {0, 0};

    public void init() {
        move[0] = ArcX.get(0);
        move[1] = ArcY.get(0);
        prevPoint[0] = ArcX.get(0);
        prevPoint[1] = ArcY.get(1);
    }

//    public void addPoint(int x, int y) {
//        xcoords.add(x);
//        ycoords.add(y);
//        repaint();
//    }
//
//    public void addArcPoint(int x, int y) {
//        ArcX.add(x);
//        ArcY.add(y);
//        repaint();
//    }

    public void clearLines() {
        xcoords.clear();
        ycoords.clear();
        ArcX.clear();
        ArcY.clear();
        repaint();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        int w = getWidth();
        int h = getHeight();
        int imageWidth = image.getWidth(this);
        int imageHeight = image.getHeight(this);
        int x = (w - imageWidth) / 2;
        int y = (h - imageHeight) / 2;
        g2.drawImage(image, x, y, this);
        g2.setPaint(Color.ORANGE);
        //g2.fill(ball);
        g.setColor(Color.ORANGE);
        int THRESHOLD = 5;
        int distance2 = 10;
        for (int i = 0; i < xcoords.size()-1; i++) {
            g.fillOval(xcoords.get(i) - 6, ycoords.get(i) - 6, 12, 12);
        }

        if (ArcX.size() >= 3) {
            for (int i = 0; i < xcoords.size() - 1; i++) { //Unless line coords match an arc point's coords
                int changeX = ArcX.get(0) - xcoords.get(i);
                int changeY = ArcY.get(0) - ycoords.get(i);
                int distance = (int) (Math.sqrt(changeX * changeX + changeY * changeY));
                if (ArcX.size() >= 6) {
                    int changeX2 = ArcX.get(3) - xcoords.get(i);
                    int changeY2 = ArcY.get(3) - ycoords.get(i);
                    distance2 = (int) (Math.sqrt(changeX2 * changeX2 + changeY2 * changeY2));
                }
                if (distance > THRESHOLD && distance2 > THRESHOLD) {
                    g.drawLine(xcoords.get(i), ycoords.get(i), xcoords.get(i + 1), ycoords.get(i + 1));
                }
            }
        }
        else {
            for (int i = 0; i < xcoords.size()-1; i++) {
                g.drawLine(xcoords.get(i), ycoords.get(i), xcoords.get(i + 1), ycoords.get(i + 1));
            }
        }

        for (int i = 0; i < ArcX.size(); i++) {
            g.fillOval(ArcX.get(i) - 6, ArcY.get(i) - 6, 12, 12);
        }

        if (ArcX.size() >= 3) {
//            g.setColor(Color.green);
//            g.fillOval((int) -move[0] - 4, (int) -move[1] - 4, 8, 8);

            Graphics2D g2d = (Graphics2D) g;
            g2d.setColor(Color.ORANGE);
            GeneralPath path = new GeneralPath();
            path.moveTo(ArcX.get(0), ArcY.get(0));
            path.quadTo(ArcX.get(1), ArcY.get(1), ArcX.get(2), ArcY.get(2));

            calculatePath();

            g2d.draw(path);
            if (ArcX.size() >= 6) {
                GeneralPath path2 = new GeneralPath();
                path2.moveTo(ArcX.get(3), ArcY.get(3));
                path2.quadTo(ArcX.get(4), ArcY.get(4), ArcX.get(5), ArcY.get(5));

//                calculatePath();

                g2d.draw(path2);
            }
            repaint();
        }
//        repaint();
    }

    public void calculateLength() {
        double vx = 2 * (ArcX.get(1) - ArcX.get(0));
        double vy = 2 * (ArcY.get(1) - ArcY.get(0));
        double wx = ArcX.get(2) - 2 * ArcX.get(1) + ArcX.get(0);
        double wy = ArcY.get(2) - 2 * ArcY.get(1) + ArcY.get(0);

        double uu = 4 * (Math.pow(wx, 2) + Math.pow(wy, 2));
        if (uu < 0.00001) {
            System.out.println(Math.sqrt(Math.pow(ArcX.get(2) - ArcX.get(0), 2) + Math.pow((ArcY.get(2) - ArcY.get(0)), 2)));
        }

        double vv = 4 * (vx * wx + vy * wy);
        double ww = Math.pow(vx, 2) + Math.pow(vy, 2);

        double t1 = (float) (2 * Math.sqrt(uu * (uu + vv + ww)));
        double t2 = 2 * uu + vv;
        double t3 = vv * vv - 4 * uu * ww;
        double t4 = (float) (2 * Math.sqrt(uu * ww));

        System.out.println(((t1 * t2 - t3 * Math.log(t2 + t1) - (vv * t4 - t3 * Math.log(vv + t4))) / (8 * Math.pow(uu, 1.5))));
    }

    public void calculatePath() {

        t += .00005;
        t %= 1;

        move[0] = (ArcX.get(0) - 2 * ArcX.get(1) + ArcX.get(2) * Math.pow(t, 2) + 2 * (ArcX.get(1) - ArcX.get(0)) * t + ArcX.get(0));
        move[1] = (ArcY.get(0) - 2 * ArcY.get(1) + ArcY.get(2) * Math.pow(t, 2) + 2 * (ArcY.get(1) - ArcY.get(0)) * t + ArcY.get(0));

        distanceDelta += Math.sqrt(Math.pow(prevPoint[0] - move[0], 2) + Math.pow(prevPoint[1] - move[1], 2));

        prevPoint[0] = move[0];
        prevPoint[1] = move[1];

        //System.out.println("X: " + (move[0] - ArcX.get(0)));
        //System.out.println("Y: " + (move[1] - ArcY.get(0)) + '\n');
    }

    private void loadImage() {
        File imageFile = new File("C:\\Users\\EJDRO\\Desktop\\Other\\Background.jpg");
        try {
            image = ImageIO.read(imageFile);
        } catch (MalformedURLException mue) {
            System.err.println(mue.getMessage());
        } catch (IOException ioe) {
            System.err.println(ioe.getMessage());
        }
    }


//    List<Integer> points = new ArrayList<Integer>(6);
    static boolean arcModeEnabled = false;
//    GraphicImagePanel imagePanel;

    private int dragIndex = NOT_DRAGGING;

    private int dragArcIndex = NOT_DRAGGING;


    private final static int NEIGHBORHOOD = 15;

    private final static int NOT_DRAGGING = -1;

    @Override
    public void mouseClicked(MouseEvent e) {
        int x = e.getX();
        int y = e.getY();
        System.out.println(x + ", " + y);



        if (!arcModeEnabled/* && ArcX.size() <3*/) {
            xcoords.add(x);
            ycoords.add(y);
            repaint();
        }
        else if (arcModeEnabled){
            ArcX.add(x);
            ArcY.add(y);

            if (ArcX.size() == 3) {
                init();
                xcoords.add(ArcX.get(0));
                ycoords.add(ArcY.get(0));
                xcoords.add(ArcX.get(2));
                ycoords.add(ArcY.get(2));
            }
            repaint();
        }
    }

    String PointType = "None";

    //Add a save button to create the csv file
    //Make csv file syntax...print 99999 for translational distance and angle when doing arc. Just print points
    //
    @Override

    public void mousePressed(MouseEvent e) { //Make multiple arcs. Start and stop lines at arcs
        dragIndex = NOT_DRAGGING;
        dragArcIndex = NOT_DRAGGING;
        int minDistance = Integer.MAX_VALUE;

        int indexOfClosestArcPoint = -1;
        int indexOfClosestPoint = -1;


        for (int i = 0; i < ArcX.size(); i++) {
            int deltaX = ArcX.get(i) - e.getX();
            int deltaY = ArcY.get(i) - e.getY();
            int distance = (int) (Math.sqrt(deltaX * deltaX + deltaY * deltaY));
            if (distance < minDistance) {
                minDistance = distance;
                PointType = "Arc";
                indexOfClosestArcPoint = i;
            }
        }

        for (int i = 0; i < xcoords.size(); i++) {
            int deltaX = xcoords.get(i) - e.getX();
            int deltaY = ycoords.get(i) - e.getY();
            int distance = (int) (Math.sqrt(deltaX * deltaX + deltaY * deltaY));
            if (distance < minDistance) {
                minDistance = distance;
                PointType = "Line";
                indexOfClosestPoint = i;
            }
        }

        if (minDistance > NEIGHBORHOOD)
            return;

        if (PointType == "Arc") {
            dragArcIndex = indexOfClosestArcPoint;
        } else if (PointType == "Line"){
            dragIndex = indexOfClosestPoint;
        }
        repaint();
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        if (dragIndex == NOT_DRAGGING && dragArcIndex == NOT_DRAGGING)
            return;

        if (dragArcIndex != NOT_DRAGGING && PointType == "Arc") {
            ArcX.set(dragArcIndex, e.getX());
            ArcY.set(dragArcIndex, e.getY());
            dragArcIndex = NOT_DRAGGING;
            repaint();

        }
        if (dragIndex != NOT_DRAGGING && PointType == "Line") {
            xcoords.set(dragIndex, e.getX());
            ycoords.set(dragIndex, e.getY());
            dragIndex = NOT_DRAGGING;
            repaint();
        }
    }

    @Override
    public void mouseEntered(MouseEvent e) {

    }

    @Override
    public void mouseExited(MouseEvent e) {

    }

    @Override
    public void mouseDragged(MouseEvent e) {
        if (dragIndex == NOT_DRAGGING && dragArcIndex == NOT_DRAGGING)
            return;

        if (dragArcIndex != NOT_DRAGGING && PointType == "Arc") {
            ArcX.set(dragArcIndex, e.getX());
            ArcY.set(dragArcIndex, e.getY());
            repaint();
        } else if (dragIndex != NOT_DRAGGING && PointType == "Line") {
            xcoords.set(dragIndex, e.getX());
            ycoords.set(dragIndex, e.getY());
            repaint();
        }
    }

    @Override
    public void mouseMoved(MouseEvent e) {

    }

    public double[] Line(int x1, int y1, int x2, int y2) {
        //Calculate real world distances in cm from graphic angles
        double xp1 = 823*x1/690;
        double yp1 = 823*(690-y1)/690;
        double xp2 = 823*x2/690;
        double yp2 = 823*(690-y2)/690;


        double[] returnValue = new double[2];
        double distance = Math.sqrt(Math.pow(xp2-xp1, 2) + Math.pow(yp2-yp1, 2));
        double direction = 0;
        if(yp2 >= yp1) {
            direction = Math.toDegrees(Math.acos((xp2-xp1)/distance));
        } else{
            direction = Math.abs(360-Math.toDegrees(Math.acos((xp2-xp1)/distance)));
        } // Cosine is better because the 0-180 acos value can be turned into 360 nicely

        returnValue[0]= distance; //Initialize this so Line returns a tuple of distance and angle
        returnValue[1]= direction;
        System.out.println("Return Value: " + returnValue[0] + ", " + returnValue[1]);
        return returnValue;
    }

    class GraphicMover extends MouseInputAdapter {
        GraphicImagePanel imagePanel;

//    private int dragIndex = NOT_DRAGGING;
//
//    private final static int NEIGHBORHOOD = 15;
//
//    private final static int NOT_DRAGGING = -1;
        public GraphicMover(GraphicImagePanel gip) {

//        ArcX.add(ArcPoint1.x);
//        ArcX.add(ArcPoint2.x);
//        ArcX.add(ArcPoint3.x);
//        ArcY.add(ArcPoint1.y);
//        ArcY.add(ArcPoint2.y);
//        ArcY.add(ArcPoint3.y);


            imagePanel = gip;

            JPanel buttonsPanel = new JPanel();
            JButton clearButton = new JButton("Clear");
            JButton doArcButton = new JButton("Make Arc");
            JButton saveFileButton = new JButton("Save File");
            imagePanel.add(buttonsPanel, BorderLayout.LINE_END);
            buttonsPanel.add(clearButton);
            buttonsPanel.add(doArcButton);
            buttonsPanel.add(saveFileButton);

            clearButton.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    imagePanel.clearLines();
//                    points.clear();
                }
            });

            doArcButton.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    if (!arcModeEnabled) {
                        doArcButton.setText("Draw Line");
                    } else {
                        doArcButton.setText("Make Arc");
                    }
                    arcModeEnabled = !arcModeEnabled;
                }
            });
            saveFileButton.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    PrintWriter pw = null;

                    try {
                        String name = new String("Test.csv"); //Make it so you can rename in the GUI later
                        pw = new PrintWriter(new File(name));
                    } catch (FileNotFoundException e1) {
                        e1.printStackTrace();
                    }

                    //For every line, append the distance and angle of the line where 0 degrees is in standard position
                    //Print from the xcoords 1 lines, then the three arc points, then the xcoords 2 lines

                    /*
                    CSV File Format:
                    * 0 = translate distance, 2 = direction to translate, 3 = direction to face
                    * 4 = Arc Point 1, 5 = Arc Point 2, 6 = Arc Point 3
                     */
                    StringBuilder sb = new StringBuilder();

                    int lineIndex = 0;
                    int THRESHOLD = 5;
                    int THRESHOLD2 = 5;


                    if (ArcX.size() >= 3) {
                        for (int i = 0; i < xcoords.size() - 1; i++) {
                            int changeX = ArcX.get(0) - xcoords.get(i);
                            int changeY = ArcY.get(0) - ycoords.get(i);
                            int distance = (int) (Math.sqrt(changeX * changeX + changeY * changeY));

                            if (distance < THRESHOLD) { //If the line isn't the fake one that starts on the arc. Is there a fake one? Or does it just add more lines than necessary?
                                THRESHOLD = distance;
                                lineIndex = i;
                            }
                        } //Do this for loop to find when the spline reaches the arc, then do another for statement to map out all the points until the arc

                        for (int i = 0; i <= lineIndex; i++) { //Print all the lines that come before the first arc
                            double[] Return = Line(xcoords.get(i), ycoords.get(i), xcoords.get(i + 1), ycoords.get(i + 1));
                            sb.append(Return[0]);
                            sb.append(',');
                            sb.append(Return[1]);
                            sb.append('\n');
                        }

                        //Print the first arc here
                        sb.append(9999999);
                        sb.append(',');
                        sb.append(9999999);
                        sb.append(',');
                        sb.append(9999999);
                        sb.append(',');

                        sb.append(ArcX.get(0) + ", " + ArcY.get(0)); //Makes a new column, making them doubles might not. Ask Joey what he would prefer
                        //Should I make the arc coords real centimeter scale values here too? Probably yeah
                        sb.append(',');
                        sb.append(ArcX.get(1) + ", " + ArcY.get(1));
                        sb.append(',');
                        sb.append(ArcX.get(2) + ", " + ArcY.get(2));
                        sb.append('\n');


                        //Do the same for the second arc
                        if (ArcX.size() >= 6) {
                            for (int i = 0; i < xcoords.size() - 1; i++) { //Print Line coords until they touch an arc, then print the arc instead
                                int changeX2 = ArcX.get(3) - xcoords.get(i);
                                int changeY2 = ArcY.get(3) - ycoords.get(i);
                                int distance2 = (int) (Math.sqrt(changeX2 * changeX2 + changeY2 * changeY2));

                                if (distance2 < THRESHOLD2) {
                                    THRESHOLD2 = distance2;
                                    lineIndex = i;
                                }
                            }
                            for (int i = 0; i <= lineIndex; i++) { //Print all the lines that come before the second arc
                                double[] Return = Line(xcoords.get(i), ycoords.get(i), xcoords.get(i + 1), ycoords.get(i + 1));
                                sb.append(Return[0]);
                                sb.append(',');
                                sb.append(Return[1]);
                                sb.append('\n');
                            }

                            //Print the second arc here
                            sb.append(9999999);
                            sb.append(',');
                            sb.append(9999999);
                            sb.append(',');
                            sb.append(9999999);
                            sb.append(',');
                            //Skip three columns for consistent syntax. Add large, unrealistic value in all of them
                            sb.append(ArcX.get(3) + ", " + ArcY.get(3));

                            sb.append(',');
                            sb.append(ArcX.get(4) + ", " + ArcY.get(4));
                            sb.append(',');
                            sb.append(ArcX.get(5) + ", " + ArcY.get(5));
                            sb.append('\n');
                        }
                    }

                    else {  //If there are no arcs
                        for (int i = 0; i < xcoords.size() - 1; i++) {
                            double[] Return = Line(xcoords.get(i), ycoords.get(i), xcoords.get(i + 1), ycoords.get(i + 1));
                            sb.append(Return[0]);
                            sb.append(',');
                            sb.append(Return[1]);
                            sb.append('\n');
                        }
                    }

                    pw.write(sb.toString());
                    pw.close();
                }
            });
        }
    }
}
