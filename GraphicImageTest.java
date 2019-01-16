package Mapping;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;
import java.net.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.event.*;

public class GraphicImageTest {
    public static void main(String[] args) {
        JFrame f = new JFrame();
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.getContentPane().add(new GraphicImagePanel());
        f.getContentPane().add(new GraphicImagePanel());
        f.setSize(1024,820);
        f.setLocation(512,440);
        f.setVisible(true);
    }
}

/*class LineMaker extends MouseAdapter {
    List<Integer> points = new ArrayList<Integer>(4);
    static boolean arcModeEnabled = false;
    GraphicImagePanel coordPanel;

    public LineMaker(GraphicImagePanel gip) {
        coordPanel = gip;
    }

    public void mouseClicked(MouseEvent e) {
        int x = e.getX();
        int y = e.getY();
        System.out.println(x + ", " + y);
        System.out.println("Success!");
        points.add(x);
        points.add(y);
        if (points.size() == 4 && !arcModeEnabled) { //And there are enough points to make a line and the code is in line mode
            int x1 = (int) (points.get(0));
            int x2 = (int) (points.get(2));
            int y1 = (int) (points.get(1));
            int y2 = (int) (points.get(3));
            System.out.println("Success!");
        }
    }
}*/
class GraphicImagePanel extends JPanel {
    Image image;
    Ellipse2D.Double ball;

    public GraphicImagePanel() {
        loadImage();
        ball = new Ellipse2D.Double(100,100,40,40);
        GraphicMover mover = new GraphicMover(this);
        addMouseListener(mover);
        addMouseMotionListener(mover);
    }
    static class Line{
        final int x1;
        final int y1;
        final int x2;
        final int y2;
        final Color color;

        public Line(int x1, int y1, int x2, int y2, Color color) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
            this.color = color;
        }
    }

    private final LinkedList<GraphicImagePanel.Line> lines = new LinkedList<GraphicImagePanel.Line>();

    public void addLine(int x1, int x2, int x3, int x4, Color color) {
        lines.add(new GraphicImagePanel.Line(x1,x2,x3,x4, color));
        repaint();
    }

    public void clearLines() {
        lines.clear();
        repaint();
    }

    @Override
    protected void paintComponent(Graphics g){
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        int w = getWidth();
        int h = getHeight();
        int imageWidth = image.getWidth(this);
        int imageHeight = image.getHeight(this);
        int x = (w - imageWidth)/2;
        int y = (h - imageHeight)/2;
        g2.drawImage(image, x, y, this);
        g2.setPaint(Color.ORANGE);
        for (GraphicImagePanel.Line line : lines) {
            //Line2D.Double line2D = new Line2D.Double(line.x1, line.y1, line.x2, line.y2);
            g.setColor(line.color);
            g.drawLine(line.x1, line.y1, line.x2, line.y2);
            //g2.fill(line2D);
        }
    }

    private void loadImage() {
        File imageFile = new File("C:\\Users\\EJDRO\\Desktop\\Other\\Background.jpg");
        try {
            image = ImageIO.read(imageFile);
        }
        catch(MalformedURLException mue) {
            System.err.println(mue.getMessage());
        }
        catch(IOException ioe) {
            System.err.println(ioe.getMessage());
        }
    }
}

class GraphicMover extends MouseInputAdapter {
    List<Integer> points = new ArrayList<Integer>(4);
    static boolean arcModeEnabled = false;
    GraphicImagePanel imagePanel;
    Point2D.Double offset;
    boolean dragging;

    public GraphicMover(GraphicImagePanel gip) {
        imagePanel = gip;
        offset = new Point2D.Double();
        dragging = false;

        JPanel buttonsPanel = new JPanel();
        JButton clearButton = new JButton("Clear");
        JButton doArcButton = new JButton("Make Arc");
        imagePanel.add(buttonsPanel, BorderLayout.LINE_END);
        buttonsPanel.add(clearButton);
        buttonsPanel.add(doArcButton);

        clearButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                imagePanel.clearLines();
                points.clear();
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
    }
    public void mouseClicked(MouseEvent e) {
        int x = e.getX();
        int y = e.getY();
        System.out.println(x + ", " + y);
        points.add(x);
        points.add(y);

        /*
        Ideas:
        Button to change from draw to move mode
        Path following: Do what curved thing did, smooth swerve
        Finally: Save points 2 and 3 to make one thing a polyline
         */
        if (points.size() == 4 && !arcModeEnabled) { //There are enough points to make a line and the code is in line mode
            int x1 = (int) (points.get(0));
            int x2 = (int) (points.get(2));
            int y1 = (int) (points.get(1));
            int y2 = (int) (points.get(3));
            System.out.println("Success!");
            imagePanel.addLine(x1, y1, x2, y2, Color.ORANGE);
            points.clear();
        }
    }
    public void mousePressed(MouseEvent e) {
        Point p = e.getPoint();
        if(imagePanel.ball.contains(p.x, p.y)) {
            offset.x = p.x - imagePanel.ball.x;
            offset.y = p.y - imagePanel.ball.y;
            dragging = true;
        }
        /*for (GraphicImagePanel.Line line : lines) {
            dragging = true;
        }*/


        //else if(imagePanel)
        //If p.x and p.y are within the threshold (5 pixels) of a line's points
    }

    public void mouseReleased(MouseEvent e) {
        dragging = false;
    }

    public void mouseDragged(MouseEvent e){
        if(dragging) {
            imagePanel.ball.x = e.getX() - offset.x;
            imagePanel.ball.y = e.getY() - offset.y;
            imagePanel.repaint();
        }
    }
}