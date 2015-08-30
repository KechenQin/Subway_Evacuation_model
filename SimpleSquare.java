package org.jbox2d.testbed.tests;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.Random;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.callbacks.RayCastCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.testbed.framework.TestbedSettings;
/**
 * @author Jia Daoyuan
 */
import org.jbox2d.testbed.framework.TestbedTest; //for Super class TestbedTest
import org.jbox2d.testbed.tests.SmokeAgent.SmokeState;

//for Super class TestbedTest

public class SimpleSquare extends TestbedTest {
	private Body m_body;
	private ArrayList<Body> m_people;
	private ArrayList<Body> smokeParticle;
	private LinkedList<SmokeAgent> smokeAgent;
	private LinkedList<PeopleAgent> m_agents;
	private Boolean m_init = false;
	private JContactListener m_listener = new JContactListener();

	private long m_worldStep;
	private long m_startEvacTime;
	private float s_origintime;
	private long m_endEvacTime;

	private BufferedWriter sr;
	private boolean needSave = false;
	private ArrayList<Incident> incidents = new ArrayList<Incident>();
	private String m_notice = "";
	private Fixture m_accidentAreaSensor;

	private boolean toggleUserSetAccident = false;
	private boolean toggleStartEvacuation = false;

	private int m_peopleNumber;

	public String notice = "";
	public static final boolean EDIT_MODE = false;
	public static final boolean SAVE_PURE_PATH = true;
	public static final boolean SHOW_WAYPOINT = false;

	public static final float SQUARE_SIZE = 0.8f;
	public static final float PILLAR_SIZE = 2.0f * SQUARE_SIZE;
	public static final float TUNNEL_LEN = 16.0f;
	public static final float TUNNEL_LEN_HOR = 14.0f;
	public static final float TUNNEL_WID = 5.0f;
	public static final float GROUND_SIZE_X = 150.f * SQUARE_SIZE;
	public static final float GROUND_SIZE_Y = 24.f * SQUARE_SIZE;
	public static final float PEOPLE_RADIUS = PILLAR_SIZE / 6.1f;
	public static final float OFFSET = 19.2f;
	// output
	public static final int SAMPLE_PRECISION = 60; // timesteps per sample
	public static final String PATH_FILE = "G://path.txt";

	public ArrayList<WayPoint> m_wayPoints2f = new ArrayList<WayPoint>(
			Arrays.asList(new WayPoint(new Vec2(62.56f, -5f), 1, 1),
					new WayPoint(new Vec2(62.56f, -7f), 1, 1), new WayPoint(
							new Vec2(62.56f, -12f), 1, 1), new WayPoint(
							new Vec2(62.56f, -14f), 1, 1),

					new WayPoint(new Vec2(57.16f, -5f), 1, 1), new WayPoint(
							new Vec2(57.16f, -7f), 1, 1), new WayPoint(
							new Vec2(57.16f, -12f), 1, 1), new WayPoint(
							new Vec2(57.16f, -14f), 1, 1),

					new WayPoint(new Vec2(10f, -5f), 1, 1), new WayPoint(
							new Vec2(10f, -7f), 1, 1), new WayPoint(new Vec2(
							10f, -12f), 1, 1), new WayPoint(
							new Vec2(10f, -14f), 1, 1),

					new WayPoint(new Vec2(111f, -5f), 1, 1), new WayPoint(
							new Vec2(111f, -7f), 1, 1), new WayPoint(new Vec2(
							111f, -12f), 1, 1), new WayPoint(new Vec2(111f,
							-14f), 1, 1),

					new WayPoint(new Vec2(17f, -5f), 2, 1), new WayPoint(
							new Vec2(17f, -7f), 2, 1), new WayPoint(new Vec2(
							17f, -12f), 2, 1), new WayPoint(
							new Vec2(17f, -14f), 2, 1),

					new WayPoint(new Vec2(48.56f, -5f), 3, 1), new WayPoint(
							new Vec2(48.56f, -7f), 3, 1), new WayPoint(
							new Vec2(48.56f, -12f), 3, 1), new WayPoint(
							new Vec2(48.56f, -14f), 3, 1),

					new WayPoint(new Vec2(38.56f, -5f), 4, 1), new WayPoint(
							new Vec2(38.56f, -7f), 4, 1), new WayPoint(
							new Vec2(38.56f, -12f), 4, 1), new WayPoint(
							new Vec2(38.56f, -14f), 4, 1),

					new WayPoint(new Vec2(33.56f, -5f), 5, 1), new WayPoint(
							new Vec2(33.56f, -7f), 5, 1), new WayPoint(
							new Vec2(33.56f, -12f), 5, 1), new WayPoint(
							new Vec2(33.56f, -14f), 5, 1),

					new WayPoint(new Vec2(48.56f, -5f), 6, 1), new WayPoint(
							new Vec2(48.56f, -7f), 6, 1), new WayPoint(
							new Vec2(48.56f, -12f), 6, 1), new WayPoint(
							new Vec2(48.56f, -14f), 6, 1),

					new WayPoint(new Vec2(70.11f, -5f), 2, 1), new WayPoint(
							new Vec2(70.11f, -7f), 2, 1), new WayPoint(
							new Vec2(70.11f, -12f), 2, 1), new WayPoint(
							new Vec2(70.11f, -14f), 2, 1),

					new WayPoint(new Vec2(80.68f, -5f), 3, 1), new WayPoint(
							new Vec2(80.68f, -7f), 3, 1), new WayPoint(
							new Vec2(80.68f, -12f), 3, 1), new WayPoint(
							new Vec2(80.68f, -14f), 3, 1),

					new WayPoint(new Vec2(87.86f, -5f), 4, 1), new WayPoint(
							new Vec2(87.86f, -7f), 4, 1), new WayPoint(
							new Vec2(87.86f, -12f), 4, 1), new WayPoint(
							new Vec2(87.86f, -14f), 4, 1),

					new WayPoint(new Vec2(90.93f, -5f), 5, 1), new WayPoint(
							new Vec2(90.93f, -7f), 5, 1), new WayPoint(
							new Vec2(90.93f, -12f), 5, 1), new WayPoint(
							new Vec2(90.93f, -14f), 5, 1),

					new WayPoint(new Vec2(26.99f, -3.5f), 7, 1), new WayPoint(
							new Vec2(26.99f, -5f), 7, 1), new WayPoint(
							new Vec2(26.99f, -14f), 7, 1), new WayPoint(
							new Vec2(26.99f, -15f), 7, 1),

					new WayPoint(new Vec2(96.4f, -3.5f), 7, 1), new WayPoint(
							new Vec2(96.4f, -5f), 7, 1), new WayPoint(new Vec2(
							96.4f, -14f), 7, 1), new WayPoint(new Vec2(96.4f,
							-15f), 7, 1),

					new WayPoint(new Vec2(22f, -5f), 8, 1), new WayPoint(
							new Vec2(22f, -7f), 8, 1), new WayPoint(new Vec2(
							22f, -12f), 8, 1), new WayPoint(
							new Vec2(22f, -14f), 8, 1),

					new WayPoint(new Vec2(104f, -5f), 8, 1), new WayPoint(
							new Vec2(104f, -7f), 8, 1), new WayPoint(new Vec2(
							104f, -12f), 8, 1), new WayPoint(new Vec2(104f,
							-14f), 8, 1),

					new WayPoint(new Vec2(22.8f, -7.5f), 9, 1), new WayPoint(
							new Vec2(22.8f, -11.3f), 9, 1),

					new WayPoint(new Vec2(102f, -7.5f), 9, 1), new WayPoint(
							new Vec2(102f, -11.3f), 9, 1),

					new WayPoint(new Vec2(23.3f, -7.5f), 10, 1), new WayPoint(
							new Vec2(23.3f, -11.3f), 10, 1),

					new WayPoint(new Vec2(101f, -7.5f), 10, 1), new WayPoint(
							new Vec2(101f, -11.3f), 10, 1)));

	public ArrayList<WayPoint> m_wayPoints = new ArrayList<WayPoint>(
			Arrays.asList(
					new WayPoint(new Vec2(23.81f, 11.41f), 1),
					new WayPoint(new Vec2(27.09f, 11.5f), 2),
					new WayPoint(new Vec2(32.76f, 11.62f), 3),
					new WayPoint(new Vec2(35.76f, 11.62f), 3),
					new WayPoint(new Vec2(40.76f, 11.62f), 5),
					new WayPoint(new Vec2(41.76f, 11.62f), 5),
					new WayPoint(new Vec2(39.56f, 11.62f), 5),
					new WayPoint(new Vec2(39.18f, 17.22f), 8), // 不考虑可达
					new WayPoint(new Vec2(40.36f, 17.22f), 8), // 不考虑可达
					new WayPoint(new Vec2(41.27f, 17.22f), 8), // 不考虑可达
					new WayPoint(new Vec2(42.33f, 17.22f), 8), // 不考虑可达

					new WayPoint(new Vec2(36.76f, 17.22f), 10), new WayPoint(
							new Vec2(29.76f, 16.5f), 11), new WayPoint(
							new Vec2(29.76f, 17.5f), 11), new WayPoint(
							new Vec2(20.76f, 17.22f), 12), new WayPoint(
							new Vec2(20.76f, 15.62f), 12), new WayPoint(
							new Vec2(20.76f, 14.22f), 12), new WayPoint(
							new Vec2(10.71f, 17.22f), 13), new WayPoint(
							new Vec2(10.71f, 15.22f), 13),

					new WayPoint(new Vec2(15.55f, 9.92f), 11), new WayPoint(
							new Vec2(15.11f, 13.22f), 12), new WayPoint(
							new Vec2(15.5f, 7.22f), 12), new WayPoint(new Vec2(
							14.5f, 3.72f), 12), new WayPoint(new Vec2(9.21f,
							9.22f), 12), new WayPoint(new Vec2(4.81f, 13.22f),
							13), new WayPoint(new Vec2(4.31f, 6.62f), 13),
					new WayPoint(new Vec2(1.71f, 9.62f), 13), new WayPoint(
							new Vec2(1.61f, 6.32f), 13), new WayPoint(new Vec2(
							3.56f, 17.22f), 15), new WayPoint(new Vec2(1.56f,
							17.22f), 15), new WayPoint(new Vec2(1.86f, 15.82f),
							15),

					new WayPoint(new Vec2(59.02f, 12.f), 1), new WayPoint(
							new Vec2(52.02f, 12.f), 2), new WayPoint(new Vec2(
							41.11f, 7.77f), 3), new WayPoint(new Vec2(47.11f,
							11.77f), 3), new WayPoint(new Vec2(50.62f, 3.35f),
							3), new WayPoint(new Vec2(53.12f, 6.20f), 3),
					new WayPoint(new Vec2(45.92f, 5.35f), 3), new WayPoint(
							new Vec2(35.12f, 6.00f), 3), new WayPoint(new Vec2(
							41.12f, 3.05f), 3),

					new WayPoint(new Vec2(58.26f, 4.45f), 1), new WayPoint(
							new Vec2(58.26f, 2.45f), 1), new WayPoint(new Vec2(
							58.26f, 7.25f), 1), new WayPoint(new Vec2(63.12f,
							3.20f), 3),
					new WayPoint(new Vec2(63.12f, 1.20f), 3), new WayPoint(
							new Vec2(52.12f, 4.20f), 3),
					new WayPoint(new Vec2(52.12f, 2.20f), 3),

					new WayPoint(new Vec2(23.26f, 7.05f), 1),
					new WayPoint(new Vec2(28.26f, 7.05f), 2),
					new WayPoint(new Vec2(31.39f, 3.79f), 3),
					new WayPoint(new Vec2(31.39f, 2.30f), 3),
					new WayPoint(new Vec2(34.39f, 2.59f), 3),
					new WayPoint(new Vec2(24.97f, 5.13f), 5),
					new WayPoint(new Vec2(24.97f, 4.16f), 5),
					new WayPoint(new Vec2(24.97f, 3.13f), 5),
					new WayPoint(new Vec2(24.97f, 2.03f), 5),
					new WayPoint(new Vec2(19.02f, 5.13f), 8), // 不考虑可达
					new WayPoint(new Vec2(19.02f, 4.16f), 8), // 不考虑可达
					new WayPoint(new Vec2(19.02f, 3.13f), 8), // 不考虑可达
					new WayPoint(new Vec2(19.02f, 2.03f), 8), // 不考虑可达
					new WayPoint(new Vec2(14.42f, 2.38f), 10), new WayPoint(
							new Vec2(14.42f, 2.38f), 10), new WayPoint(
							new Vec2(10.42f, 4.70f), 12), new WayPoint(
							new Vec2(10.42f, 2.31f), 12), new WayPoint(
							new Vec2(6.29f, 2.23f), 13), new WayPoint(new Vec2(
							6.29f, 3.53f), 13), new WayPoint(new Vec2(3.30f,
							1.35f), 15), new WayPoint(new Vec2(1.30f, 1.35f),
							15),

					new WayPoint(new Vec2(61.02f, 12.4f), 1), new WayPoint(
							new Vec2(62.62f, 10.4f), 2), new WayPoint(new Vec2(
							65.82f, 7.4f), 2), new WayPoint(new Vec2(65.82f,
							2.5f), 3),
					new WayPoint(new Vec2(73.32f, 12.11f), 3), new WayPoint(
							new Vec2(73.32f, 10.11f), 3), new WayPoint(
							new Vec2(73.32f, 8.11f), 3), new WayPoint(new Vec2(
							73.32f, 6.11f), 3), new WayPoint(new Vec2(73.32f,
							4.11f), 3),
					new WayPoint(new Vec2(73.32f, 2.11f), 3),

					new WayPoint(new Vec2(70.55f, 12.4f), 1), new WayPoint(
							new Vec2(68.42f, 11.45f), 2),

					new WayPoint(new Vec2(69.47f, 3.03f), 3), new WayPoint(
							new Vec2(81.62f, 3.57f), 2), new WayPoint(new Vec2(
							91.22f, 3.55f), 4), new WayPoint(new Vec2(91.22f,
							2.35f), 4),
					new WayPoint(new Vec2(91.22f, 4.65f), 4),

					new WayPoint(new Vec2(97.02f, 5.28f), 5), new WayPoint(
							new Vec2(97.02f, 4.16f), 5), new WayPoint(new Vec2(
							97.02f, 3.17f), 5), new WayPoint(new Vec2(97.02f,
							2.08f), 5), new WayPoint(new Vec2(103.41f, 5.28f),
							8), // 不考虑可达
					new WayPoint(new Vec2(103.41f, 4.16f), 8), // 不考虑可达
					new WayPoint(new Vec2(103.41f, 3.17f), 8), // 不考虑可达
					new WayPoint(new Vec2(103.41f, 2.08f), 8), // 不考虑可达
					new WayPoint(new Vec2(110.f, 3.72f), 10), new WayPoint(
							new Vec2(113.f, 3.72f), 11), new WayPoint(new Vec2(
							116.f, 3.42f), 13), new WayPoint(new Vec2(118.67f,
							6.42f), 12), new WayPoint(new Vec2(117.63f, 1.72f),
							15),

					new WayPoint(new Vec2(98.22f, 11.4f), 1), new WayPoint(
							new Vec2(90.f, 11.72f), 2), new WayPoint(new Vec2(
							83.59f, 6.42f), 3), new WayPoint(new Vec2(98.22f,
							7.05f), 1), new WayPoint(new Vec2(90.f, 7.05f), 2),

					new WayPoint(new Vec2(81.2f, 11.72f), 5), new WayPoint(
							new Vec2(81.9f, 11.72f), 5), new WayPoint(new Vec2(
							82.8f, 11.72f), 5), new WayPoint(new Vec2(81.14f,
							16.72f), 8), // 不考虑可达
					new WayPoint(new Vec2(82.0f, 16.72f), 8), // 不考虑可达
					new WayPoint(new Vec2(82.83f, 16.72f), 8),// 不考虑可达
					new WayPoint(new Vec2(86.92f, 18.01f), 10), new WayPoint(
							new Vec2(86.92f, 15.01f), 10), new WayPoint(
							new Vec2(94.78f, 18.01f), 11), new WayPoint(
							new Vec2(94.78f, 16.01f), 11), new WayPoint(
							new Vec2(94.78f, 14.01f), 11), new WayPoint(
							new Vec2(100.68f, 18.31f), 12), new WayPoint(
							new Vec2(100.68f, 16.31f), 12), new WayPoint(
							new Vec2(100.68f, 14.31f), 12), new WayPoint(
							new Vec2(108.5f, 17.6f), 13), new WayPoint(
							new Vec2(108.5f, 15.6f), 13), new WayPoint(
							new Vec2(108.5f, 14.6f), 13), new WayPoint(
							new Vec2(114.2f, 13.06f), 13), new WayPoint(
							new Vec2(104.66f, 12.6f), 12), new WayPoint(
							new Vec2(117.12f, 18.09f), 15), new WayPoint(
							new Vec2(118.12f, 17.09f), 15), new WayPoint(
							new Vec2(116.12f, 18.89f), 15), new WayPoint(
							new Vec2(112.04f, 8.5f), 12),

					new WayPoint(new Vec2(3.47f, 41.86f), 20), new WayPoint(
							new Vec2(1.47f, 40.16f), 20), new WayPoint(
							new Vec2(1.39f, -17.71f), 20), new WayPoint(
							new Vec2(3.39f, -19.71f), 20), new WayPoint(
							new Vec2(117.69f, 39.28f), 20), new WayPoint(
							new Vec2(119.69f, 37.28f), 20), new WayPoint(
							new Vec2(117.09f, -22.71f), 20), new WayPoint(
							new Vec2(119.09f, -20.71f), 20),

					new WayPoint(new Vec2(-13.47f, 40.16f), 25), new WayPoint(
							new Vec2(-13.47f, -17.71f), 25), new WayPoint(
							new Vec2(132.69f, 36.28f), 25), new WayPoint(
							new Vec2(132.69f, -21.71f), 25)

			));

	private Body m_ground;

	private void drawLineOnGround(Vec2 vec1, Vec2 vec2) {
		EdgeShape edge = new EdgeShape();
		edge.set(vec1, vec2);
		m_ground.createFixture(edge, 0);
	}

	private void drawBox(float length, float width, Vec2 mid) {

		BodyDef pillarDef = new BodyDef();
		pillarDef.type = BodyType.STATIC;
		pillarDef.position.set(mid);

		PolygonShape pillarShape = new PolygonShape();
		pillarShape.setAsBox(length / 2, width / 2);

		Body pillar = this.getWorld().createBody(pillarDef);
		pillar.createFixture(pillarShape, 0.0f);

	}

	private void drawBox(Vec2 lefttop, Vec2 rightbottom) {

		BodyDef pillarDef = new BodyDef();
		pillarDef.type = BodyType.STATIC;
		pillarDef.position.set(new Vec2((rightbottom.x + lefttop.x) / 2,
				(lefttop.y + rightbottom.y) / 2));

		PolygonShape pillarShape = new PolygonShape();
		pillarShape.setAsBox((rightbottom.x - lefttop.x) / 2,
				(lefttop.y - rightbottom.y) / 2);

		Body pillar = this.getWorld().createBody(pillarDef);
		pillar.createFixture(pillarShape, 0.0f);

	}

	private void drawPolygon(ArrayList<Vec2> veclist) {
		for (int i = 0; i < veclist.size(); i++) {
			if (i + 1 != veclist.size()) {
				drawLineOnGround(veclist.get(i), veclist.get(i + 1));
			}
		}
	}

	private void drawCircleEdge(Vec2 origin, float radius) {
		int edges = 20;
		ArrayList<Vec2> circle = new ArrayList<Vec2>();
		float degree = MathUtils.TWOPI / edges;

		for (int i = 1; i <= edges + 1; i++) {
			Vec2 vec = new Vec2();
			vec.y = radius * MathUtils.sin(degree * i);
			vec.x = radius * MathUtils.cos(degree * i);
			vec.addLocal(origin);
			circle.add(vec);
		}

		this.drawPolygon(circle);
	}

	@Override
	public void initTest(boolean deserialized) {
		if (deserialized)
			return;
		try {
			this.sr = new BufferedWriter(new FileWriter(PATH_FILE));
			this.sr.write(String.format("%d\r\n", SAMPLE_PRECISION / 60)); // 点间距代表的时间，
																			// 60
																			// /
																			// 60hz
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		{
			this.notice = "e: Start Evacuation \ns：Set Random Accident and Evacuate \nq: toggle User Set Accident(Left Click)\np：Stop and Save Path\n";
			mouseDown(new Vec2());
			toggleStartEvacuation = false;
			toggleUserSetAccident = false;
			needSave = false;
			this.m_worldStep = 0;
			this.m_startEvacTime = 0;
			this.s_origintime = 0;
			this.m_endEvacTime = 0;
			this.m_people = new ArrayList<Body>();
			this.m_agents = new LinkedList<PeopleAgent>();
			this.smokeAgent = new LinkedList<SmokeAgent>();
			this.smokeParticle = new ArrayList<Body>();
			this.m_peopleNumber = 300;

		}
		getWorld().setGravity(new Vec2(0.0f, 0.0f));
		// getWorld().setContactListener(m_listener);

		// 创建线类物体
		{
			BodyDef bd = new BodyDef();
			bd.position.set(new Vec2(0.0f, 0.0f));
			Body ground = this.getWorld().createBody(bd);
			m_ground = ground;

			float offset = 3.0f;

			drawLineOnGround(new Vec2(0.f + TUNNEL_WID, 0.f), new Vec2(
					GROUND_SIZE_X - TUNNEL_WID, 0));
			drawLineOnGround(new Vec2(0.f + TUNNEL_WID, GROUND_SIZE_Y),
					new Vec2(GROUND_SIZE_X - TUNNEL_WID, GROUND_SIZE_Y));
			drawLineOnGround(new Vec2(0.f, 0.f - TUNNEL_LEN), new Vec2(0.f,
					GROUND_SIZE_Y + TUNNEL_LEN + offset));
			drawLineOnGround(new Vec2(TUNNEL_WID, 0.f), new Vec2(TUNNEL_WID,
					0.f - TUNNEL_LEN - TUNNEL_WID));
			drawLineOnGround(new Vec2(TUNNEL_WID, GROUND_SIZE_Y), new Vec2(
					TUNNEL_WID, GROUND_SIZE_Y + TUNNEL_LEN + TUNNEL_WID
							+ offset));

			drawLineOnGround(new Vec2(0.f - TUNNEL_LEN_HOR, GROUND_SIZE_Y
					+ TUNNEL_LEN + TUNNEL_WID + offset), new Vec2(TUNNEL_WID,
					GROUND_SIZE_Y + TUNNEL_LEN + TUNNEL_WID + offset));
			drawLineOnGround(new Vec2(0.f - TUNNEL_LEN_HOR, GROUND_SIZE_Y
					+ TUNNEL_LEN + offset), new Vec2(0.0f, GROUND_SIZE_Y
					+ TUNNEL_LEN + offset));
			drawLineOnGround(new Vec2(0.f - TUNNEL_LEN_HOR, 0.0f - TUNNEL_LEN
					- TUNNEL_WID), new Vec2(TUNNEL_WID, 0.0f - TUNNEL_LEN
					- TUNNEL_WID));
			drawLineOnGround(new Vec2(0.f - TUNNEL_LEN_HOR, 0.0f - TUNNEL_LEN),
					new Vec2(0.0f, 0.0f - TUNNEL_LEN));

			// left tunnel end
			drawLineOnGround(new Vec2(0.f - TUNNEL_LEN_HOR, 0.0f - TUNNEL_LEN
					- TUNNEL_WID), new Vec2(0.f - TUNNEL_LEN_HOR,
					0.0f - TUNNEL_LEN));

			drawLineOnGround(new Vec2(0.f - TUNNEL_LEN_HOR, GROUND_SIZE_Y
					+ TUNNEL_LEN + TUNNEL_WID + offset), new Vec2(
					0.f - TUNNEL_LEN_HOR, GROUND_SIZE_Y + TUNNEL_LEN + offset));

			drawLineOnGround(
					new Vec2(GROUND_SIZE_X, 0.f - TUNNEL_LEN - offset),
					new Vec2(GROUND_SIZE_X, GROUND_SIZE_Y + TUNNEL_LEN));
			drawLineOnGround(new Vec2(GROUND_SIZE_X - TUNNEL_WID, 0.f),
					new Vec2(GROUND_SIZE_X - TUNNEL_WID, 0.f - TUNNEL_LEN
							- TUNNEL_WID - offset));

			drawLineOnGround(
					new Vec2(GROUND_SIZE_X - TUNNEL_WID, GROUND_SIZE_Y),
					new Vec2(GROUND_SIZE_X - TUNNEL_WID, GROUND_SIZE_Y
							+ TUNNEL_LEN + TUNNEL_WID));
			drawLineOnGround(new Vec2(GROUND_SIZE_X + TUNNEL_LEN_HOR,
					GROUND_SIZE_Y + TUNNEL_LEN + TUNNEL_WID), new Vec2(
					GROUND_SIZE_X - TUNNEL_WID, GROUND_SIZE_Y + TUNNEL_LEN
							+ TUNNEL_WID));
			drawLineOnGround(new Vec2(GROUND_SIZE_X + TUNNEL_LEN_HOR,
					GROUND_SIZE_Y + TUNNEL_LEN), new Vec2(GROUND_SIZE_X,
					GROUND_SIZE_Y + TUNNEL_LEN));
			drawLineOnGround(new Vec2(GROUND_SIZE_X + TUNNEL_LEN_HOR, 0.0f
					- TUNNEL_LEN - TUNNEL_WID - offset), new Vec2(GROUND_SIZE_X
					- TUNNEL_WID, 0.0f - TUNNEL_LEN - TUNNEL_WID - offset));
			drawLineOnGround(new Vec2(GROUND_SIZE_X + TUNNEL_LEN_HOR, 0.0f
					- TUNNEL_LEN - offset), new Vec2(GROUND_SIZE_X, 0.0f
					- TUNNEL_LEN - offset));
			// right tunnel end
			drawLineOnGround(new Vec2(GROUND_SIZE_X + TUNNEL_LEN_HOR, 0.0f
					- TUNNEL_LEN - TUNNEL_WID - offset), new Vec2(GROUND_SIZE_X
					+ TUNNEL_LEN_HOR, 0.0f - TUNNEL_LEN - offset));
			drawLineOnGround(new Vec2(GROUND_SIZE_X + TUNNEL_LEN_HOR,
					GROUND_SIZE_Y + TUNNEL_LEN + TUNNEL_WID), new Vec2(
					GROUND_SIZE_X + TUNNEL_LEN_HOR, GROUND_SIZE_Y + TUNNEL_LEN));

			drawLineOnGround(new Vec2(22.404f, 5.688f), new Vec2(22.404f,
					6.381f));
			drawLineOnGround(new Vec2(22.404f, 1.516f), new Vec2(22.404f, 0));

			drawLineOnGround(new Vec2(99.2f, 5.688f), new Vec2(99.2f, 6.381f));
			drawLineOnGround(new Vec2(99.2f, 1.516f), new Vec2(99.2f, 0));

			// 左一电梯楼梯
			drawLineOnGround(new Vec2(29.2f, 6.381f), new Vec2(20.372f, 6.381f));
			drawLineOnGround(new Vec2(20.372f, 8.701f), new Vec2(20.372f,
					6.381f));
			drawLineOnGround(new Vec2(20.372f, 8.701f), new Vec2(29.2f, 8.701f));

			drawLineOnGround(new Vec2(28.125f, 10.473f), new Vec2(20.4f,
					10.473f));
			drawLineOnGround(new Vec2(20.4f, 12.801f), new Vec2(20.4f, 10.473f));
			drawLineOnGround(new Vec2(28.125f, 12.801f), new Vec2(20.4f,
					12.801f));

			 ArrayList<Vec2> a=new ArrayList<Vec2>();
			 a.add(new Vec2(28.125f, 12.801f));
			 a.add(new Vec2(28.41f, 12.801f));
			 a.add(new Vec2(28.41f, 14.367f));
			 a.add(new Vec2(38.714f, 14.367f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(42.886f, 14.367f));
			 a.add(new Vec2(53.2f, 14.367f));
			 a.add(new Vec2(54.0f, 13.601f));
			 a.add(new Vec2(60.0f, 13.601f));
			 a.add(new Vec2(60.0f, 11.281f));
			 a.add(new Vec2(54.0f, 11.281f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(60.0f, 13.601f));
			 a.add(new Vec2(60.4f, 13.601f));
			 a.add(new Vec2(61.2f, 14.401f));
			 a.add(new Vec2(64.0f, 14.401f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(68.0f, 14.401f));
			 a.add(new Vec2(68.4f, 14.401f));
			 a.add(new Vec2(68.8f, 14.001f));
			 a.add(new Vec2(73.604f, 14.01f));
			 a.add(new Vec2(74.0f, 14.401f));
			 a.add(new Vec2(80.714f, 14.401f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(83.286f, 14.401f));
//			 a.add(new Vec2(85f, 14.401f));
//			
//			 drawPolygon(a);
//			 a.clear();
//			
//			
//			
//			 a.add(new Vec2(85.8f, 14.401f));
//			 a.add(new Vec2(86f, 14.401f));
//			 drawPolygon(a);
//			 a.clear();
//			
//			
//			 a.add(new Vec2(86.8f, 14.401f));
//			 a.add(new Vec2(87f, 14.401f));
//			 drawPolygon(a);
//			 a.clear();
//			
//			
//			 a.add(new Vec2(87.8f, 14.401f));
//			 a.add(new Vec2(88f, 14.401f));
//			 drawPolygon(a);
//			 a.clear();
//			
//			 a.add(new Vec2(88.8f, 14.401f));
//			 a.add(new Vec2(89f, 14.401f));
//			 drawPolygon(a);
//			 a.clear();
//			
//			 a.add(new Vec2(89.8f, 14.401f));
			 a.add(new Vec2(90f, 14.401f));
			 drawPolygon(a);
			 a.clear();
			
			
			
			
			 a.add(new Vec2(90f, 14.401f));
			 a.add(new Vec2(93.6f, 14.401f));
			 a.add(new Vec2(93.6f, 12.801f));
			 a.add(new Vec2(101.61f, 12.801f));
			 a.add(new Vec2(101.61f, 10.491f));
			 a.add(new Vec2(93.8f, 10.491f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(92.8f, 8.701f));
			 a.add(new Vec2(101.6f,8.701f));
			 a.add(new Vec2(101.6f,8.701f));
			 a.add(new Vec2(101.6f,6.373f));
			 a.add(new Vec2(92.8f,6.373f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(101.61f, 10.491f));
			 a.add(new Vec2(101.6f,8.701f));
			 drawPolygon(a);
			 a.clear();
			

		}
		{

			 ArrayList<Vec2> a=new ArrayList<Vec2>();
			 //下半层
			 a.add(new Vec2(5.545f, -3.128f));
			 a.add(new Vec2(115.545f,-3.128f));
			 a.add(new Vec2(115.545f, -16.128f));
			 a.add(new Vec2(5.545f,-16.128f));
			 a.add(new Vec2(5.545f, -3.128f));
			 drawPolygon(a);
			 a.clear();
			
			
			 a.add(new Vec2(24.04f, -8.759f));
			 a.add(new Vec2(24.04f, -10.379f));
			 drawPolygon(a);
			 a.clear();
			
			 a.add(new Vec2(23.765f, -12.597f));
			 a.add(new Vec2(28.2f, -12.597f));
			 a.add(new Vec2(28.2f, -6.554f));
			 a.add(new Vec2(23.765f, -6.554f));
			 drawPolygon(a);
			 a.clear();
			
			
			 a.add(new Vec2(28.2f+72f, -12.597f));
			 a.add(new Vec2(23.765f+72f, -12.597f));
			 a.add(new Vec2(23.765f+72f, -6.554f));
			 a.add(new Vec2(28.2f+72f, -6.554f));
			 drawPolygon(a);
			 a.clear();
			//
			// 下半层
			float pillars_y1 = 9.6f;
			float pillars_x1[] = { 12.0f, 20.0f, 27.2f, 35.2f, 44.0f, 52.0f,
					60.0f, 68.0f, 76.0f, 84.0f, 92.0f, 99.2f, 107.2f };

			for (int i = 0; i < pillars_x1.length; i++) {
				BodyDef pillarDef = new BodyDef();
				pillarDef.type = BodyType.STATIC;
				pillarDef.position.set(new Vec2(pillars_x1[i] + SQUARE_SIZE,
						pillars_y1 - OFFSET));

				PolygonShape pillarShape = new PolygonShape();
				pillarShape.setAsBox(PILLAR_SIZE / 2, PILLAR_SIZE / 2);

				Body pillar = this.getWorld().createBody(pillarDef);
				pillar.createFixture(pillarShape, 0.0f);

			}

			// 左广告牌
			drawBox(0.3f, 1.1f, new Vec2(7.802f, -7.872f));
			drawBox(0.3f, 1.1f, new Vec2(7.802f, -9.509f));
			drawBox(0.3f, 1.1f, new Vec2(7.802f, -11.156f));

			drawBox(0.56f, 0.25f, new Vec2(12.805f, -8.61f));
			drawBox(2.4f, 1.2f, new Vec2(40.87f, -9.579f));
			drawBox(2.4f, 1.2f, new Vec2(72.87f, -9.579f));

			drawBox(2.4f, 2.4f, new Vec2(48.422f, -6.09f));

			drawBox(1.0f, 1.0f, new Vec2(31.997f, -9.639f));

			drawBox(1.0f, 1.0f, new Vec2(88.986f, -9.639f));

			// 右广告牌
			drawBox(0.3f, 1.1f, new Vec2(113.799f, -7.872f));
			drawBox(0.3f, 1.1f, new Vec2(113.799f, -9.509f));
			drawBox(0.3f, 1.1f, new Vec2(113.799f, -11.156f));

			drawBox(0.5f, 1.3f, new Vec2(77.924f, -9.602f));
			drawBox(0.5f, 1.3f, new Vec2(106.937f, -9.602f));
			drawBox(0.5f, 1.3f, new Vec2(51.735f, -9.602f));
		}

		// 创建柱子等实心物体
		{
			float pillars_y = 9.6f;
			float pillars_x[] = { 4.0f, 12.0f, 20.0f, 27.2f, 35.2f, 44.0f,
					52.0f, 60.0f, 68.0f, 76.0f, 84.0f, 92.0f, 99.2f, 107.2f,
					115.2f };

			for (int i = 0; i < pillars_x.length; i++) {
				BodyDef pillarDef = new BodyDef();
				pillarDef.type = BodyType.STATIC;
				pillarDef.position.set(new Vec2(pillars_x[i] + SQUARE_SIZE,
						pillars_y));

				PolygonShape pillarShape = new PolygonShape();
				pillarShape.setAsBox(PILLAR_SIZE / 2, PILLAR_SIZE / 2);

				Body pillar = this.getWorld().createBody(pillarDef);
				pillar.createFixture(pillarShape, 0.0f);

			}

			// 上下闸门
			drawBox(new Vec2(21.295f, 5.688f), new Vec2(23.489f, 5.516f));
			drawBox(new Vec2(21.295f, 4.688f), new Vec2(23.489f, 4.516f));
			drawBox(new Vec2(21.295f, 3.688f), new Vec2(23.489f, 3.516f));
			drawBox(new Vec2(21.295f, 2.688f), new Vec2(23.489f, 2.516f));
			drawBox(new Vec2(21.295f, 1.688f), new Vec2(23.489f, 1.516f));
			// 售票
			drawBox(new Vec2(64.0f, 14.801f), new Vec2(68.0f, 11.617f));
			// 上闸门
			drawBox(new Vec2(80.714f, 15.495f), new Vec2(80.886f, 13.315f));
			drawBox(new Vec2(81.514f, 15.495f), new Vec2(81.686f, 13.315f));
			drawBox(new Vec2(82.289f, 15.495f), new Vec2(82.47f, 13.315f));
			drawBox(new Vec2(83.114f, 15.495f), new Vec2(83.286f, 13.315f));
			// 右闸门
			drawBox(new Vec2(98.11f, 5.688f), new Vec2(100.288f, 5.516f));
			drawBox(new Vec2(98.11f, 4.688f), new Vec2(100.288f, 4.516f));
			drawBox(new Vec2(98.11f, 3.688f), new Vec2(100.288f, 3.516f));
			drawBox(new Vec2(98.11f, 2.688f), new Vec2(100.288f, 2.516f));
			drawBox(new Vec2(98.11f, 1.688f), new Vec2(100.288f, 1.516f));
			// 左上闸门
			drawBox(new Vec2(38.714f, 15.49f), new Vec2(38.886f, 13.31f));
			drawBox(new Vec2(39.714f, 15.49f), new Vec2(39.886f, 13.31f));
			drawBox(new Vec2(40.714f, 15.49f), new Vec2(40.886f, 13.31f));
			drawBox(new Vec2(41.714f, 15.49f), new Vec2(41.886f, 13.31f));
			drawBox(new Vec2(42.714f, 15.49f), new Vec2(42.886f, 13.31f));
			// 书柜
			drawBox(new Vec2(68.8f, 14.801f), new Vec2(73.6f, 14.001f));
			// 自动售货机等
			drawBox(new Vec2(101.819f, 12.828f), new Vec2(102.216f, 11.881f));
			drawBox(new Vec2(101.777f, 11.727f), new Vec2(102.366f, 11.127f));
			drawBox(new Vec2(101.761f, 10.971f), new Vec2(102.351f, 10.371f));
			drawBox(new Vec2(101.692f, 10.059f), new Vec2(102.492f, 8.959f));
			drawBox(new Vec2(101.692f, 8.878f), new Vec2(102.492f, 7.778f));
			drawBox(new Vec2(101.692f, 7.69f), new Vec2(102.492f, 6.41f));

			drawBox(new Vec2(39.026f, 0.356f), new Vec2(39.826f, 0));
			drawBox(new Vec2(63.061f, 0.356f), new Vec2(63.861f, 0));
			drawBox(new Vec2(83.658f, 0.356f), new Vec2(84.458f, 0));

			drawBox(new Vec2(69.6f, 8.401f), new Vec2(72.0f, 6.001f));

		}

		{
			this.createPeople(m_peopleNumber);
		}

	}

	private void createPeople(int count) {
		{
			Vec2[][] forbiddenArea = {
					{ new Vec2(19.91f, 8.72f), new Vec2(29.15f, 10.54f) },
					{ new Vec2(68.71f, 13.81f), new Vec2(73.62f, 14.87f) },
					{ new Vec2(91.87f, 8.57f), new Vec2(101.67f, 10.58f) },
					{ new Vec2(101.57f, 6.32f), new Vec2(102.51f, 12.91f) } };

			Random rand = new Random(Calendar.getInstance().getTimeInMillis());
			for (int i = 0; i < count; i++) {
				Vec2 initMove = new Vec2(MathUtils.randomFloat(-1.0f, 1.0f),
						MathUtils.randomFloat(-1.0f, 1.0f));
				BodyDef peopleDef = new BodyDef();
				peopleDef.linearDamping = 0.0f;
				peopleDef.angularDamping = 1.0f;
				peopleDef.type = BodyType.DYNAMIC;
				peopleDef.linearVelocity = initMove;

				// for()
				float v = rand.nextFloat();
				String type = null;
				if (v < 0.1) {
					type = "kid";
				} else if (v < 0.5 && v >= 0.1) {
					type = "woman";
				} else if (v < 0.92 && v >= 0.5) {
					type = "man";
				} else {
					type = "elder";
				}
				PeopleAgent agent = ModelFactory.createPeople(type, i, this);

				Vec2 position = new Vec2(rand.nextFloat()
						* (GROUND_SIZE_X - PILLAR_SIZE) + PILLAR_SIZE / 2,
						rand.nextFloat() * (GROUND_SIZE_Y - PILLAR_SIZE)
								+ PILLAR_SIZE / 2); // new Vec2(16.72f,11.87f)
													// ;//
				boolean flag = false;
				for (Vec2[] square : forbiddenArea) {
					if (position.x > square[0].x && position.x < square[1].x
							&& position.y > square[0].y
							&& position.y < square[1].y) {
						flag = true;
						break;
					}
				}
				if (flag) {
					i--;
					continue;
				}

				peopleDef.position = position;
				// PeopleAgent agent = new PeopleAgent(i,this);
				m_agents.add(agent);

				Body people = this.getWorld().createBody(peopleDef);
				agent.init(people);
				people.setUserData(agent);

				CircleShape peopleShape = new CircleShape();
				peopleShape.setRadius(PEOPLE_RADIUS);

				people.createFixture(peopleShape, 3.0f);

				this.m_people.add(people);

				// acallback.init(people);
				// getWorld().raycast(acallback, new Vec2(23.81f, 11.41f),new
				// Vec2(18.72f,11.87f));
			}
		}
	}

	public void createSubsmoke(SmokeAgent smokeagent) {
		Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		BodyDef subsmoke = new BodyDef();
		subsmoke.type = BodyType.DYNAMIC;
		subsmoke.position.set(smokeagent.s_body.getPosition());
		Body subparticle = this.getWorld().createBody(subsmoke);
		float x = MathUtils.randomFloat(rand, 1f, 3f);
		float y = MathUtils.randomFloat(rand, 0f, 0f);
		subparticle.setLinearVelocity(new Vec2(x, y));
		// subparticle.setLinearVelocity(new Vec2(1.6f, 0f));
		SmokeAgent subagent = new SmokeAgent(this, s_origintime);
		subagent.setDensity(smokeagent.getDensity() / 2);
		subagent.setDifFac(smokeagent.getDifFac() / 1.8f);
		subagent.setParticleNum(smokeagent.getParticleNum() / 2);
		smokeAgent.add(subagent);
		subagent.init(subparticle);
		subparticle.setUserData(subagent);
		PolygonShape shape = new PolygonShape();
		shape.setAsBox(0.125f, 0.125f);
		FixtureDef fd = new FixtureDef();
		fd.shape = shape;
		fd.restitution = 0.2f;
		fd.density = 1.0f;
		//fd.filter.groupIndex = 3;
		fd.friction = 0.0f;
		subparticle.createFixture(fd);
		smokeParticle.add(subparticle);
		smokeagent.state = SmokeState.NORMAL;

	}

	public void createSmoke(Vec2 point, int particleNum) {

		for (int i = 0; i < particleNum; i++) {
			Random rand = new Random(Calendar.getInstance().getTimeInMillis());
			BodyDef smoke = new BodyDef();
			smoke.type = BodyType.DYNAMIC;
			smoke.position.set(point);
			Body particle = this.getWorld().createBody(smoke);
			float x = MathUtils.randomFloat(rand, 1f, 3f);
			float y = MathUtils.randomFloat(rand, 0f, 0f);
			particle.setLinearVelocity(new Vec2(x, y));
			// particle.setLinearVelocity(new Vec2(1.6f,0f));
			SmokeAgent agent = new SmokeAgent(i, this, s_origintime);
			smokeAgent.add(agent);
			agent.init(particle);
			particle.setUserData(agent);
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.125f, 0.125f);
			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 1.0f;
			fd.filter.groupIndex = -10;
			fd.restitution = 0.2f;
			fd.friction = 0.0f;

			particle.createFixture(fd);
			smokeParticle.add(particle);

		}
		// for(int i=0;i<particleNum; i++) {
		// BodyDef smoke = new BodyDef();
		// smoke.type = BodyType.DYNAMIC;
		// smoke.position.set(point);
		// Body particle = this.getWorld().createBody(smoke);
		// particle.setLinearVelocity(new Vec2(-1f,-1f));
		// SmokeAgent agent= new SmokeAgent(i,this);
		// smokeAgent.add(agent);
		// agent.init(particle);
		// particle.setUserData(agent);
		// PolygonShape shape = new PolygonShape();
		// shape.setAsBox(0.125f, 0.125f);
		// particle.createFixture(shape, 1.0f);
		// smokeParticle.add(particle);
		//
		//
		// }
		// for(int i=0;i<particleNum; i++) {
		// BodyDef smoke = new BodyDef();
		// smoke.type = BodyType.DYNAMIC;
		// smoke.position.set(point);
		// Body particle = this.getWorld().createBody(smoke);
		// particle.setLinearVelocity(new Vec2(1f,-1f));
		// SmokeAgent agent= new SmokeAgent(i,this);
		// smokeAgent.add(agent);
		// agent.init(particle);
		// particle.setUserData(agent);
		// PolygonShape shape = new PolygonShape();
		// shape.setAsBox(0.125f, 0.125f);
		// particle.createFixture(shape, 1.0f);
		// smokeParticle.add(particle);
		//
		//
		// }
		// for(int i=0;i<particleNum; i++) {
		// BodyDef smoke = new BodyDef();
		// smoke.type = BodyType.DYNAMIC;
		// smoke.position.set(point);
		// Body particle = this.getWorld().createBody(smoke);
		// particle.setLinearVelocity(new Vec2(-1f,1f));
		// SmokeAgent agent= new SmokeAgent(i,this);
		// smokeAgent.add(agent);
		// agent.init(particle);
		// particle.setUserData(agent);
		// PolygonShape shape = new PolygonShape();
		// shape.setAsBox(0.125f, 0.125f);
		// particle.createFixture(shape, 1.0f);
		// smokeParticle.add(particle);

		// }
	}

	private void createRandomAccident() {
		Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		float radius = MathUtils.randomFloat(rand, 3.0f, 5.0f);
		Vec2 position = new Vec2(MathUtils.randomFloat(rand, 29.0f, 90.f),
				MathUtils.randomFloat(rand, 2.f, 11.0f));
		this.incidents.add(0, new Incident(position, radius, this.m_worldStep
				* 1.0f / SAMPLE_PRECISION));
		this.createAccidentArea();
	}

	private void createAccidentArea() {

		CircleShape shape = new CircleShape();
		Incident incident = this.incidents.get(0);
		shape.m_p.set(incident.position.x, incident.position.y);
		shape.m_radius = incident.radius + 0.1f;

		FixtureDef fixture = new FixtureDef();
		fixture.isSensor = true;
		fixture.shape = shape;

		 if(this.m_accidentAreaSensor != null)
		 m_ground.destroyFixture(m_accidentAreaSensor);
		 this.m_accidentAreaSensor = m_ground.createFixture(fixture);

		 this.drawCircleEdge(incident.position, incident.radius);
		//createSmoke(incident.position,1000);

		this.notice += String.format("事故发生在坐标: %f ,%f ， 影响半径 ： %f\n",
				incident.position.x, incident.position.y, incident.radius);
	}

	// Implement contact listener.
	public void beginContact(Contact contact) {
		Fixture fixtureA = contact.getFixtureA();
		Fixture fixtureB = contact.getFixtureB();
		Object userDataA = fixtureA.getBody().getUserData();
		Object userDataB = fixtureB.getBody().getUserData();

		if (userDataA instanceof PeopleAgent
				&& userDataB instanceof PeopleAgent) {
			if (((PeopleAgent) userDataA).state == PeopleAgent.PeopleState.EVACUATED
					|| ((PeopleAgent) userDataB).state == PeopleAgent.PeopleState.EVACUATED) {
				((PeopleAgent) userDataA).Die();
				((PeopleAgent) userDataB).Die();
				((PeopleAgent) userDataA).state = ((PeopleAgent) userDataB).state = PeopleAgent.PeopleState.EVACUATED;
			}
		}

		if (fixtureA == m_accidentAreaSensor) {
			Object userData = fixtureB.getBody().getUserData();
			if (userData != null && userData instanceof PeopleAgent) {
				((PeopleAgent) userData).Die();
			}
		}

		if (fixtureB == m_accidentAreaSensor) {
			Object userData = fixtureA.getBody().getUserData();
			if (userData != null && userData instanceof PeopleAgent) {
				((PeopleAgent) userData).Die();
			}
		}
	}

	@Override
	public String getTestName() {
		return "Simple Evacuation Demo";
	}

	@Override
	public void mouseDown(Vec2 p) {
		super.mouseDown(p);
		if (toggleUserSetAccident) {
			Vec2 postion = new Vec2(p.x, p.y);
			float radius = 5.0f;
			this.incidents.add(0, new Incident(postion, radius,
					this.m_worldStep * 1.0f / SAMPLE_PRECISION));
			s_origintime = m_worldStep / 60f;
			this.createAccidentArea();

			// System.out.println("world"+s_origintime/60f);
			return;
		}

		m_notice = notice + "\nx: " + p.x + "         y:  " + p.y;
	}

	@Override
	public void keyPressed(char argKeyChar, int argKeyCode) {
		if (argKeyChar == 's') {

			this.createRandomAccident();
			for (Body people : m_people) {
				if (((PeopleAgent) people.m_userData).state == PeopleAgent.PeopleState.NORMAL) {
					((PeopleAgent) people.m_userData).state = PeopleAgent.PeopleState.EVACUATING;
					continue;
				}
			}
			toggleStartEvacuation = true;
			m_startEvacTime = m_worldStep;

		} else if (argKeyChar == 'q') {
			toggleUserSetAccident = !toggleUserSetAccident;

		} else if (argKeyChar == 'e') {
			toggleStartEvacuation = !toggleStartEvacuation;
			if (toggleStartEvacuation)
				m_startEvacTime = m_worldStep;

			for (Body body : this.m_people) {
				if (((PeopleAgent) body.m_userData).state == PeopleAgent.PeopleState.EVACUATING
						&& !toggleStartEvacuation) {
					((PeopleAgent) body.m_userData).state = PeopleAgent.PeopleState.NORMAL;
					continue;
				}
				if (((PeopleAgent) body.m_userData).state == PeopleAgent.PeopleState.NORMAL
						&& toggleStartEvacuation) {
					((PeopleAgent) body.m_userData).state = PeopleAgent.PeopleState.EVACUATING;
					continue;
				}
			}
		} else if (argKeyChar == 'p') {
			this.needSave = true;
		} else if (argKeyChar == '=') {
			this.m_peopleNumber += 10;
			this.createPeople(10);
		} else if (argKeyChar == '-') {

			for (int i = 1; i <= 10; i++) {
				this.getWorld().destroyBody(m_people.get(m_peopleNumber - i));
				this.m_people.remove(m_peopleNumber - i);
				this.m_agents.remove(m_peopleNumber - i);

			}
			this.m_peopleNumber -= 10;
		}
	}

	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		m_worldStep++;
		{
			addTextLine("People Num: " + this.m_peopleNumber);
		}

		{
			addTextLine("");
			addTextLine("==Help==");
		}
		String[] info = m_notice.split("\n");
		for (String item : info) {
			addTextLine(item);
		}

		{
			addTextLine("");
			addTextLine("==Status==");
			addTextLine("Toggle People Evacuation "
					+ (this.toggleStartEvacuation ? "On" : "Off"));
			addTextLine("Toggle User Set Accident "
					+ (this.toggleUserSetAccident ? "On" : "Off"));

			addTextLine(String.format("Sim World Time: %f sec",
					this.m_worldStep * 1.0f / SAMPLE_PRECISION));
			addTextLine(String.format("Start Evac Time: %f sec",
					this.m_startEvacTime * 1.0f / SAMPLE_PRECISION));
			addTextLine(String.format("End Evac Time: %f sec",
					this.m_endEvacTime * 1.0f / SAMPLE_PRECISION));
		}

		if (SHOW_WAYPOINT) {
			for (WayPoint p : m_wayPoints2f) {
				getDebugDraw().drawPoint(p.Vec, 2.0f,
						new Color3f(0.4f, 0.9f, 0.4f));
			}
			for (WayPoint p : m_wayPoints) {
				getDebugDraw().drawPoint(p.Vec, 2.0f,
						new Color3f(0.4f, 0.9f, 0.4f));
			}
		}

		if (this.m_endEvacTime == 0) {
			boolean isEvacuationFinish = true;
			for (Body people : m_people) {
				if (((PeopleAgent) people.m_userData).state == PeopleAgent.PeopleState.EVACUATING
						|| ((PeopleAgent) people.m_userData).state == PeopleAgent.PeopleState.NORMAL)
					isEvacuationFinish = false;
			}
			if (isEvacuationFinish)
				this.m_endEvacTime = m_worldStep;
		}

		if (needSave) {
			try {

				sr.write(String.format("%d\r\n", this.incidents.size()));
				for (Incident incident : this.incidents) {
					sr.write(String.format("%f 0.0 %f %f %f\r\n",
							incident.position.x, incident.position.y,
							incident.radius, incident.time));
				}

				for (Body people : m_people) {
					this.savePeoplePath(people);

					if (((PeopleAgent) (people.getUserData())).IsRemoved)
						continue;
					this.getWorld().destroyBody(people);
				}

				this.sr.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			m_people.clear();
		}

		for (int i = 0; i < smokeParticle.size(); i++) {
			// System.out.println(smokeParticle.size());
			Body particle = smokeParticle.get(i);
			SmokeAgent agent = (SmokeAgent) (particle.getUserData());
			agent.setOrigintime(s_origintime / 60f);
			// System.out.println(agent.getOrigintime());

			agent.updateDen();
			agent.updateState();

			if (agent.state == SmokeState.NORMAL) {
				agent.updateSpd();

			}

			if (agent.state == SmokeState.DIVIDE) {
				agent.updateSpd();
				createSubsmoke(agent);

			} else if (agent.state == SmokeState.DEAD) {
				this.getWorld().destroyBody(particle);
			} else if (agent.state == SmokeState.STATIC) {
				agent.setDensity(agent.getDensity() / 100000);
				
				if(agent.getDensity()==0){
					this.getWorld().destroyBody(particle);
				}
			}

		}

		for (int i = 0; i < m_people.size(); i++) {
			Body people = m_people.get(i);
			PeopleAgent agent = (PeopleAgent) (people.getUserData());

			if (agent.IsRemoved)
				continue;

			agent.UpdateToNextPosition();
			// System.out.println(12);
			if (agent.state == PeopleAgent.PeopleState.EVACUATED) {
				agent.IsRemoved = true;
				this.getWorld().destroyBody(people);
			}
		}

	}

	private void savePeoplePath(Body people) {
		try {
			sr.write(((PeopleAgent) people.getUserData()).GetPathString());
			sr.write("\r\n");
			sr.flush();
		} catch (Exception e) {
			System.out.println("123");
		}
	}
}

class SmokeAgent {
	enum SmokeState {
		NORMAL, DIVIDE, DEAD, STATIC
	};

	private double particleNum = 4500;
	private float difFac = 0.1f; // 这是烟雾扩散系数，需要再研究。
	private float timestep = 0f;
	private float ttl;
	private float origintime;
	private double density;
	protected Body s_body;
	protected World s_world;
	private int s_id;
	private float distance;
	public Vec2 originposition = null;
	public static Random rand = new Random(Calendar.getInstance()
			.getTimeInMillis());
	private JSmokeQueryCallback s_simpleQueryCallback = new JSmokeQueryCallback();

	public SmokeState state = SmokeState.NORMAL;

	public SmokeAgent(SimpleSquare scene) {
		s_world = scene.getWorld();

	}

	public SmokeAgent(SimpleSquare scene, float origintime) {
		s_world = scene.getWorld();
		this.origintime = origintime;
	}

	public SmokeAgent(int id, SimpleSquare scene) {
		s_id = id;
		s_world = scene.getWorld();
	}

	public SmokeAgent(int id, SimpleSquare scene, float origintime) {
		s_id = id;
		s_world = scene.getWorld();
		this.origintime = origintime;
		// System.out.println(origintime);

	}

	public void init(Body body) {
		this.s_body = body;
		originposition = s_body.getPosition().clone();

	}

	public void updateSpd() {
		float x = MathUtils.randomFloat(rand, 1f, 3f);
		float y = MathUtils.randomFloat(rand, 0f, 0f);
		s_body.setLinearVelocity(new Vec2(x, y));
	}

	public void updateDen() {
		timestep++;
		ttl = (timestep / 60f);
		distance = MathUtils.distance(s_body.getPosition(), originposition);

		// System.out.println("时间"+ttl);
		// System.out.println("距离"+distance);
		// 这是高科技计算密度的公式，我们假设扩散系数为常数

		// density=1.0/(Math.pow(difFac,
		// 3.0)*Math.pow(Math.E,(1.0/Math.pow(difFac,2.0))));
		if (this.state != SmokeState.STATIC) {

			density = (Math
					.pow(Math.E, -(Math.pow(distance, 2) / (4f * ttl * Math
							.pow(difFac, 2)))))
					/ (Math.pow(2f * difFac * Math.sqrt(Math.PI * ttl), 3));

		}

		// density= ((1f/Math.pow((2f*difFac*Math.sqrt((Math.PI*ttl))),3f)) *
		// Math.pow((Math.E),( Math.pow(distance, 2f)/
		// (-4f*Math.pow(difFac,2f)*distance))));
		// System.out.println("系数"+(difFac-0.41f));
		// System.out.println("密度"+density);
	}

	public void updateState() {
		this.getDampingBeforeObstacles();

		// System.out.println("density："+(int)density);
		// System.out.println("parti："+(int)particleNum);
		if ((int) density < (int) particleNum / 2 && (int) density > 0
				&& this.state != SmokeState.STATIC) {

			this.state = SmokeState.DIVIDE;
			particleNum = particleNum / 2;
		}
		if (density == 0) {
			this.state = SmokeState.DEAD;
		}

	}

	protected void getDampingBeforeObstacles() {
		Vec2 position = s_body.getPosition();
		float angle = s_body.getTransform().q.getAngle() + s_body.getAngle();
		float radius = 0.125f;

		AABB smokeAABB=null;
		this.s_simpleQueryCallback.SetBody(s_body);
		




		if (angle >= -MathUtils.QUARTER_PI && angle < MathUtils.QUARTER_PI) {
			smokeAABB = new AABB(
					position.sub(new Vec2(0.f - radius, 0.001f)),
					position.add(new Vec2(radius, 0.001f)));
		} else if (angle >= MathUtils.QUARTER_PI
				&& angle <= 3 * MathUtils.QUARTER_PI) {
			smokeAABB = new AABB(
					position.sub(new Vec2(0.001f, 0.f - radius)),
					position.add(new Vec2(0.001f, radius)));
		} else if (angle >= -3 * MathUtils.QUARTER_PI
				&& angle <= -MathUtils.QUARTER_PI) {
			smokeAABB = new AABB(position.sub(new Vec2(0.001f, 
					+ radius)), position.add(new Vec2(0.001f, 0.f - radius)));
		} else {
			smokeAABB = new AABB(position.sub(new Vec2(
					+ radius, 0.001f)), position.add(new Vec2(0.f - radius,
							0.001f)));
		}

		s_world.queryAABB(s_simpleQueryCallback, smokeAABB);
		// if( s_simpleQueryCallback.hasObj ){
		// break;
		// }

		// damping *= ( count *1.0f) / (max * 1.0f);
		// return (float) Math.pow((double)damping , 0.4);
	}

	public double getParticleNum() {
		return particleNum;
	}

	public void setParticleNum(double particleNum) {
		this.particleNum = particleNum;
	}

	public float getDifFac() {
		return difFac;
	}

	public void setDifFac(float difFac) {
		this.difFac = difFac;
	}

	public float getTtl() {
		return ttl;
	}

	public void setTtl(float ttl) {
		this.ttl = ttl;
	}

	public double getDensity() {
		return density;
	}

	public void setDensity(double density) {
		this.density = density;
	}

	public float getOrigintime() {
		return origintime;
	}

	public void setOrigintime(float origintime) {
		this.origintime = origintime;
	}

	// public void divide(){
	// for(int i=0;i<particleNum; i++) {
	// BodyDef smoke = new BodyDef();
	// smoke.type = BodyType.DYNAMIC;
	// smoke.position.set(position);
	// Body particle = s_world.createBody(smoke);
	// particle.setLinearVelocity(new Vec2(1f,1f));
	// SmokeAgent agent= new SmokeAgent(i,this);
	// smokeAgent.add(agent);
	// agent.init(particle);
	// particle.setUserData(agent);
	// PolygonShape shape = new PolygonShape();
	// shape.setAsBox(0.125f, 0.125f);
	// particle.createFixture(shape, 1.0f);
	// smokeParticle.add(particle);
	// }
	//
	// }
}

// people body's userData
class PeopleAgent {
	enum PeopleState {
		NORMAL, EVACUATING, EVACUATED, STUCK, DEAD
	};

	public static final float PEOPLE_WALK_SPEED = 2.0f;
	public static final float PEOPLE_RUN_SPEED = 6.0f;
	public static final float PEOPLE_SAFE_LENGTH = 0.2f;
	public static final float PEOPLE_SAFE_WIDTH = 0.2f;
	public static final float PEOPLE_ROT_SPEED = 0.55f * MathUtils.PI;
	public static final int SAMPLE_PRECISION = 60;

	public static Random rand = new Random(Calendar.getInstance()
			.getTimeInMillis());
	protected boolean slowDownBeforeObstacle = true;
	protected boolean flockBehave = false;

	protected Body m_body;
	private int m_id;
	protected int m_stepCount = 0;

	public boolean IsRemoved = false;
	public boolean Finished = false;
	public WayPoint target = null;
	public WayPoint current = null;
	public Vec2 targetVec = null;
	public PeopleState state = PeopleState.NORMAL;

	protected World m_world;
	private ArrayList<WayPoint> m_candidatePoints;
	private ArrayList<WayPoint> wayPoints;
	private Vec2 oldPosition = null;
	private int jamCounter = 0; // 是否人被卡住
	public LinkedList<Vec2> m_path = new LinkedList<Vec2>();
	private WayPointComparer comp = new WayPointComparer();
	private JRayCastClosestCallback acallback = new JRayCastClosestCallback();
	protected JQueryCallback m_queryCallback = new JQueryCallback();
	private JSimpleQueryCallback m_simpleQueryCallback = new JSimpleQueryCallback();

	public PeopleAgent(int id, SimpleSquare scene) {
		m_id = id;
		wayPoints = scene.m_wayPoints;
		m_candidatePoints = (ArrayList<WayPoint>) scene.m_wayPoints.clone();
		m_world = scene.getWorld();
	}

	public void init(Body body) {
		m_body = body;
		oldPosition = m_body.getPosition().clone();
		current = new WayPoint(m_body.getPosition(), 0);
		target = new WayPoint(m_body.getPosition(), 0);
		targetVec = target.Vec.clone();
	}

	private void filterCandidatePoints() {
		ArrayList<WayPoint> back = new ArrayList<WayPoint>();
		for (WayPoint p : m_candidatePoints) {
			if (p.Weigh > target.Weigh)
				back.add(p);
		}
		this.m_candidatePoints = back;
	}

	private ArrayList<WayPoint> getPointsOfWeigh(int weigh) {
		ArrayList<WayPoint> list = new ArrayList<WayPoint>();
		for (WayPoint p : wayPoints) {
			if (p.Weigh == weigh)
				list.add(p);
		}
		return list;
	}

	private void stuckStateCheck() {
		int threshold = (int) (0.8f * SimpleSquare.SAMPLE_PRECISION);
		float distThreshold = 0.07f;

		if (!Finished
				&& MathUtils.distance(m_body.getPosition(), oldPosition) <= distThreshold) {
			this.jamCounter++;
		} else {
			oldPosition = m_body.getPosition().clone();
			this.jamCounter = 0;
		}

		if (this.jamCounter > threshold) {
			// this.state = PeopleState.STUCK;
			solvePeopleStuck();
			this.jamCounter = 0;
		}
	}

	private void solvePeopleStuck() {
		ArrayList<WayPoint> reachable = getReachInPoints(wayPoints);

		if (reachable.size() > 0) {
			comp.init(new WayPoint(current.Vec, 1));
			target.ChooseCount--;
			target = Collections.min(reachable, comp);
			target.ChooseCount++;
		} else {
			this.Die();
		}

		ArrayList<WayPoint> lowWeighPt = getPointsOfWeigh(current.Weigh);
		m_candidatePoints.addAll(lowWeighPt);

		float offsetLimit = 0.50f;
		float x = MathUtils.randomFloat(rand, -offsetLimit, offsetLimit);
		float y = MathUtils.randomFloat(rand, -offsetLimit, offsetLimit);
		target.ChooseCount++;
		targetVec = target.Vec.clone();
		targetVec.x += x;
		targetVec.y += y;

	}

	private ArrayList<WayPoint> getReachInPoints(ArrayList<WayPoint> candidate) {
		ArrayList<WayPoint> list = new ArrayList<WayPoint>();
		for (WayPoint p : candidate) {
			acallback.init(m_body);
			m_world.raycast(acallback, p.Vec, current.Vec);
			if (!acallback.m_hit) {
				list.add(p);
			}
		}
		return list;
	}

	private void calcNextTarget() {
		if (current.Weigh == 25) {
			this.Finished = true;
			return;
		}

		m_candidatePoints.remove(current);
		filterCandidatePoints();
		target.ChooseCount--;
		if (current.Weigh != 55) {
			ArrayList<WayPoint> reachin = getReachInPoints(m_candidatePoints);
			if (reachin.size() == 0) {
				if (current.Weigh == 0)
					this.Finished = true;
				return;
			}
			target = Collections.min(reachin, comp);
		} else {
			ArrayList<WayPoint> reachin = getPointsOfWeigh(8);
			target = Collections.min(reachin, comp);
		}

		float offsetLimit = 0.50f;
		float x = MathUtils.randomFloat(rand, -offsetLimit, offsetLimit);
		float y = MathUtils.randomFloat(rand, -offsetLimit, offsetLimit);
		target.ChooseCount++;
		targetVec = target.Vec.clone();
		targetVec.x += x;
		targetVec.y += y;
	}

	private void updateCurrent() {
		current = new WayPoint(m_body.getPosition(), target.Weigh);
		comp.init(current);
	}

	public void evacuate() {
		float threshold = 0.1f;
		Vec2 pos = m_body.getPosition();
		float distance = MathUtils.distance(pos, targetVec);
		Vec2 speed = m_body.getLinearVelocity();
		m_body.setLinearVelocity(new Vec2());
		if (distance > threshold
				&& MathUtils.distance(speed, new Vec2()) < PEOPLE_RUN_SPEED) {
			speed = new Vec2((targetVec.x - pos.x) / distance
					* PEOPLE_RUN_SPEED, (targetVec.y - pos.y) / distance
					* PEOPLE_RUN_SPEED);
		}

		float damping = 1.0f;
		if (slowDownBeforeObstacle) {
			damping = getDampingBeforeObstacles();
		}

		if (flockBehave) {

		}

		speed = speed.mulLocal(damping);
		float angle = this.getAngleFromVec(speed);
		m_body.setLinearVelocity(speed);
		m_body.setTransform(pos, angle);
	}

	protected float getDampingBeforeObstacles() {
		float damping = 1.0f;
		int max = 5;
		int count = 1;
		float base = 0.25f, wid = 0.3f;
		Vec2 position = m_body.getPosition();
		float angle = m_body.getTransform().q.getAngle() + m_body.getAngle();
		AABB peopleAABB = null;
		float radius = 0.28f;

		for (; count < max; count++) {

			this.m_simpleQueryCallback.init(m_body);

			if (angle >= -MathUtils.QUARTER_PI && angle < MathUtils.QUARTER_PI) {
				peopleAABB = new AABB(
						position.sub(new Vec2(0.f - radius, wid)),
						position.add(new Vec2(base * count + radius, wid)));
			} else if (angle >= MathUtils.QUARTER_PI
					&& angle <= 3 * MathUtils.QUARTER_PI) {
				peopleAABB = new AABB(
						position.sub(new Vec2(wid, 0.f - radius)),
						position.add(new Vec2(wid, base * count + radius)));
			} else if (angle >= -3 * MathUtils.QUARTER_PI
					&& angle <= -MathUtils.QUARTER_PI) {
				peopleAABB = new AABB(position.sub(new Vec2(wid, base * count
						+ radius)), position.add(new Vec2(wid, 0.f - radius)));
			} else {
				peopleAABB = new AABB(position.sub(new Vec2(base * count
						+ radius, wid)), position.add(new Vec2(0.f - radius,
						wid)));
			}

			m_world.queryAABB(m_simpleQueryCallback, peopleAABB);
			if (m_simpleQueryCallback.hasObj) {
				break;
			}

		}

		damping *= (count * 1.0f) / (max * 1.0f);
		return (float) Math.pow((double) damping, 0.4);
	}

	private void rotToTarget(Body body, Vec2 target) {
		float threshold = 0.01f;
		Vec2 pos = body.getPosition();
		float distance = MathUtils.distance(pos, target);
		float angle = body.getAngle();
		float pi = MathUtils.PI;
		float targetCos = (target.x - pos.x) / distance;

		if (angle - 0.0f > 0.00001f) {
			angle = angle - ((int) (angle / (2 * pi))) * (2 * pi);
			if (angle > pi)
				angle -= 2 * pi;
		} else if (angle - 0.0f < -0.00001f) {
			angle = angle + ((int) (-angle / (2 * pi))) * (2 * pi);
			if (angle < -pi)
				angle += 2 * pi;
		}

		float targetAngle = (float) Math.acos(targetCos)
				* (target.y - pos.y > 0 ? 1.0f : -1.0f);

		float tmp = targetAngle - angle;
		if (tmp > threshold && tmp < pi || tmp < -pi)
			body.setAngularVelocity(PEOPLE_ROT_SPEED);
		else if (tmp < -threshold && tmp > -pi || tmp > pi)
			body.setAngularVelocity(-PEOPLE_ROT_SPEED);
		else
			body.setAngularVelocity(0.0f);
	}

	protected float getAngleFromVec(Vec2 vec) {
		return (float) Math.acos(vec.x / vec.length())
				* (vec.y < 0 ? -1.0f : 1.0f);
	}

	/**
	 * get angle of tow vectors
	 * 
	 * @param vec1
	 *            : vector rotation from
	 * @param vac2
	 *            : vector rotation to
	 * @return angle of vec1 rotation to vec2, range from -PI to PI
	 * 
	 * @author Jia
	 */
	private float getAngleTwoVecs(Vec2 vec1, Vec2 vec2) {
		float cos = Vec2.dot(vec1, vec2) / (vec1.length() * vec2.length());
		float angle = (float) Math.acos(cos)
				* (vec1.y - vec2.y > 0 ? 1.0f : -1.0f);
		return angle;
	}

	/**
	 * assgin a random speed(vector) to a body
	 * 
	 * @author Jia
	 */
	protected void randomMove() {
		Vec2 speed = new Vec2(MathUtils.randomFloat(-1.0f, 1.0f),
				MathUtils.randomFloat(-1.0f, 1.0f));
		float angle = this.getAngleFromVec(speed);
		m_body.setLinearVelocity(speed);
		m_body.setTransform(m_body.getPosition(), angle);
	}

	public void UpdateToNextPosition() {
		if (this.state == PeopleState.EVACUATING) {
			this.Update();
			this.evacuate();
			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);

		} else if (this.state == PeopleState.NORMAL) {
			if (MathUtils.randomFloat(0.0f, 1.0f) > 0.99)
				this.randomMove();

			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);
		} else if (this.state == PeopleState.DEAD) {

		}

		if (++m_stepCount % SAMPLE_PRECISION == 0) {
			this.addPath();
			m_stepCount = 0;
		}

	}

	public void Update() {
		float threshold = 0.8f;
		if (Finished
				&& MathUtils.distance(m_body.getPosition(), target.Vec) < threshold) {
			this.Die();
			this.state = PeopleState.EVACUATED;

			return;
		}
		if (MathUtils.distance(m_body.getPosition(), target.Vec) < threshold) {
			updateCurrent();
			calcNextTarget();
		} else {
			stuckStateCheck();
		}

	}

	public void Die() {
		this.addPath();
		this.m_body.m_linearVelocity.set(new Vec2());
		this.m_body.m_linearDamping = 1000.0f;
		this.state = PeopleState.DEAD;
	}

	protected void addPath() {
		if (this.state != PeopleState.DEAD)
			m_path.add(m_body.getPosition().clone());
	}

	public LinkedList<Vec2> GetPath() {
		return m_path;
	}

	public String GetPathXML() {
		StringBuilder str = new StringBuilder();
		str.append("<AddonsPathNode name=\"path_" + m_id + "\">\r\n");
		for (int i = 0; i < m_path.size(); i++) {
			Vec2 pos = m_path.get(i);
			str.append(String
					.format("<AddonsPointNode tx=\"%f\" ty=\"0.0\" tz=\"%f\" id=\"%d\" name=\"path%d_pathNode%d\"/>\r\n",
							pos.x, pos.y, i, m_id, i));
		}
		str.append("</AddonsPathNode>");
		return str.toString();
	}

	public String GetPathString() {
		StringBuilder str = new StringBuilder();
		for (int i = 0; i < m_path.size(); i++) {
			Vec2 pos = m_path.get(i);
			str.append(String.format("%f %f %f\r\n", pos.x, this.getZAxis(pos),
					pos.y));
		}

		if (this.state == PeopleState.DEAD) {
			str.append("DIE\r\n");
		} else if (this.state == PeopleState.EVACUATED) {
			str.append("Evacuated\r\n");
		}
		return str.toString();
	}

	public float getZAxis(Vec2 position) {
		float floorHeight = 4.169f;
		Vec2[][] stairs = {
				{ new Vec2(20.41f, 10.51f), new Vec2(28.58f, 12.73f) },
				{ new Vec2(20.38f, 6.38f), new Vec2(29.09f, 8.76f) },
				{ new Vec2(59.94f, 13.58f), new Vec2(53.86f, 11.38f) },
				{ new Vec2(101.58f, 12.80f), new Vec2(93.47f, 10.55f) },
				{ new Vec2(101.58f, 8.67f), new Vec2(92.6f, 6.38f) } };
		Vec2[] theStair = null;
		for (Vec2[] stair : stairs) {
			boolean cond1 = position.x >= stair[0].x
					&& position.y >= stair[0].y && position.x <= stair[1].x
					&& position.y <= stair[1].y;
			boolean cond2 = position.x <= stair[0].x
					&& position.y <= stair[0].y && position.x >= stair[1].x
					&& position.y >= stair[1].y;
			if (cond1 || cond2) {
				theStair = stair;
				break;
			}
		}

		if (theStair != null) {
			float z = 0f;

			z = MathUtils.map(position.x, theStair[0].x, theStair[1].x,
					-floorHeight, 0);

			return z;
		} else {
			return 0.0f;
		}
	}
}

class ManAgent extends PeopleAgent {
	public ManAgent(int id, SimpleSquare scene) {
		super(id, scene);
		// TODO Auto-generated constructor stub
	}

	public static final float PEOPLE_WALK_SPEED = 1.55f;
	public static final float PEOPLE_RUN_SPEED = 4.65f;
	public static final float PEOPLE_SAFE_LENGTH = MathUtils.randomFloat(
			0.383f, 0.486f);
	public static final float PEOPLE_SAFE_WIDTH = MathUtils.randomFloat(0.176f,
			0.261f);
	public static final float PEOPLE_ROT_SPEED = 0.55f * MathUtils.PI;

	public String GetPathString() {
		StringBuilder str = new StringBuilder();
		for (int i = 0; i < m_path.size(); i++) {
			Vec2 pos = m_path.get(i);
			str.append(String.format("%f %f %f\r\n man", pos.x,
					this.getZAxis(pos), pos.y));
		}

		if (this.state == PeopleState.DEAD) {
			str.append("DIE\r\n");
		} else if (this.state == PeopleState.EVACUATED) {
			str.append("Evacuated\r\n");
		}
		return str.toString();
	}

	public void UpdateToNextPosition() {
		if (this.state == PeopleState.EVACUATING) {
			this.Update();
			this.evacuate();
			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);

		} else if (this.state == PeopleState.NORMAL) {
			if (MathUtils.randomFloat(0.0f, 1.0f) > 0.99)
				this.randomMove();

			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);
		} else if (this.state == PeopleState.DEAD) {

		}

		if (++m_stepCount % SAMPLE_PRECISION == 0) {
			this.addPath();
			m_stepCount = 0;
		}

	}

	public void evacuate() {
		float threshold = 0.1f;
		Vec2 pos = m_body.getPosition();
		float distance = MathUtils.distance(pos, targetVec);
		Vec2 speed = m_body.getLinearVelocity();
		m_body.setLinearVelocity(new Vec2());
		if (distance > threshold
				&& MathUtils.distance(speed, new Vec2()) < PEOPLE_RUN_SPEED) {
			speed = new Vec2((targetVec.x - pos.x) / distance
					* PEOPLE_RUN_SPEED, (targetVec.y - pos.y) / distance
					* PEOPLE_RUN_SPEED);
		}

		float damping = 1.0f;
		if (slowDownBeforeObstacle) {
			damping = getDampingBeforeObstacles();
		}

		if (flockBehave) {

		}

		speed = speed.mulLocal(damping);
		float angle = this.getAngleFromVec(speed);
		m_body.setLinearVelocity(speed);
		m_body.setTransform(pos, angle);
	}
}

class WomanAgent extends PeopleAgent {
	public WomanAgent(int id, SimpleSquare scene) {
		super(id, scene);
		// TODO Auto-generated constructor stub
	}

	public static final float PEOPLE_WALK_SPEED = 1.45f;
	public static final float PEOPLE_RUN_SPEED = 4.35f;
	public static final float PEOPLE_SAFE_LENGTH = MathUtils.randomFloat(
			0.347f, 0.458f);
	public static final float PEOPLE_SAFE_WIDTH = MathUtils.randomFloat(0.159f,
			0.260f);
	public static final float PEOPLE_ROT_SPEED = 0.55f * MathUtils.PI;

	public String GetPathString() {
		StringBuilder str = new StringBuilder();
		for (int i = 0; i < m_path.size(); i++) {
			Vec2 pos = m_path.get(i);
			str.append(String.format("%f %f %f\r\n woman", pos.x,
					this.getZAxis(pos), pos.y));
		}

		if (this.state == PeopleState.DEAD) {
			str.append("DIE\r\n");
		} else if (this.state == PeopleState.EVACUATED) {
			str.append("Evacuated\r\n");
		}
		return str.toString();
	}

	public void UpdateToNextPosition() {
		if (this.state == PeopleState.EVACUATING) {
			this.Update();
			this.evacuate();
			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);

		} else if (this.state == PeopleState.NORMAL) {
			if (MathUtils.randomFloat(0.0f, 1.0f) > 0.99)
				this.randomMove();

			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);
		} else if (this.state == PeopleState.DEAD) {

		}

		if (++m_stepCount % SAMPLE_PRECISION == 0) {
			this.addPath();
			m_stepCount = 0;
		}

	}

	public void evacuate() {
		float threshold = 0.1f;
		Vec2 pos = m_body.getPosition();
		float distance = MathUtils.distance(pos, targetVec);
		Vec2 speed = m_body.getLinearVelocity();
		m_body.setLinearVelocity(new Vec2());
		if (distance > threshold
				&& MathUtils.distance(speed, new Vec2()) < PEOPLE_RUN_SPEED) {
			speed = new Vec2((targetVec.x - pos.x) / distance
					* PEOPLE_RUN_SPEED, (targetVec.y - pos.y) / distance
					* PEOPLE_RUN_SPEED);
		}

		float damping = 1.0f;
		if (slowDownBeforeObstacle) {
			damping = getDampingBeforeObstacles();
		}

		if (flockBehave) {

		}

		speed = speed.mulLocal(damping);
		float angle = this.getAngleFromVec(speed);
		m_body.setLinearVelocity(speed);
		m_body.setTransform(pos, angle);
	}
}

class ElderAgent extends PeopleAgent {
	public ElderAgent(int id, SimpleSquare scene) {
		super(id, scene);
		// TODO Auto-generated constructor stub
	}

	public static final float PEOPLE_WALK_SPEED = 0.8f;
	public static final float PEOPLE_RUN_SPEED = 2.4f;
	public static final float PEOPLE_SAFE_LENGTH = 0.2f;
	public static final float PEOPLE_SAFE_WIDTH = 0.2f;
	public static final float PEOPLE_ROT_SPEED = 0.55f * MathUtils.PI;

	public String GetPathString() {
		StringBuilder str = new StringBuilder();
		for (int i = 0; i < m_path.size(); i++) {
			Vec2 pos = m_path.get(i);
			str.append(String.format("%f %f %f\r\n elder", pos.x,
					this.getZAxis(pos), pos.y));
		}

		if (this.state == PeopleState.DEAD) {
			str.append("DIE\r\n");
		} else if (this.state == PeopleState.EVACUATED) {
			str.append("Evacuated\r\n");
		}
		return str.toString();
	}

	public void UpdateToNextPosition() {
		if (this.state == PeopleState.EVACUATING) {
			this.Update();
			this.evacuate();
			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);

		} else if (this.state == PeopleState.NORMAL) {
			if (MathUtils.randomFloat(0.0f, 1.0f) > 0.99)
				this.randomMove();

			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);
		} else if (this.state == PeopleState.DEAD) {

		}

		if (++m_stepCount % SAMPLE_PRECISION == 0) {
			this.addPath();
			m_stepCount = 0;
		}

	}

	public void evacuate() {
		float threshold = 0.1f;
		Vec2 pos = m_body.getPosition();
		float distance = MathUtils.distance(pos, targetVec);
		Vec2 speed = m_body.getLinearVelocity();
		m_body.setLinearVelocity(new Vec2());
		if (distance > threshold
				&& MathUtils.distance(speed, new Vec2()) < PEOPLE_RUN_SPEED) {
			speed = new Vec2((targetVec.x - pos.x) / distance
					* PEOPLE_RUN_SPEED, (targetVec.y - pos.y) / distance
					* PEOPLE_RUN_SPEED);
		}

		float damping = 1.0f;
		if (slowDownBeforeObstacle) {
			damping = getDampingBeforeObstacles();
		}

		if (flockBehave) {

		}
		// System.out.println(damping);
		speed = speed.mulLocal(damping);
		float angle = this.getAngleFromVec(speed);
		// System.out.println(speed);
		m_body.setLinearVelocity(speed);
		m_body.setTransform(pos, angle);
	}
}

class KidAgent extends PeopleAgent {
	public KidAgent(int id, SimpleSquare scene) {
		super(id, scene);
		// TODO Auto-generated constructor stub
	}

	public static final float PEOPLE_WALK_SPEED = 1.6f;
	public static final float PEOPLE_RUN_SPEED = 4.8f;
	public static final float PEOPLE_SAFE_LENGTH = 0.2f;
	public static final float PEOPLE_SAFE_WIDTH = 0.2f;
	public static final float PEOPLE_ROT_SPEED = 0.55f * MathUtils.PI;

	public String GetPathString() {
		StringBuilder str = new StringBuilder();
		for (int i = 0; i < m_path.size(); i++) {
			Vec2 pos = m_path.get(i);
			str.append(String.format("%f %f %f\r\n kid", pos.x,
					this.getZAxis(pos), pos.y));
		}

		if (this.state == PeopleState.DEAD) {
			str.append("DIE\r\n");
		} else if (this.state == PeopleState.EVACUATED) {
			str.append("Evacuated\r\n");
		}
		return str.toString();
	}

	public void UpdateToNextPosition() {
		if (this.state == PeopleState.EVACUATING) {
			this.Update();
			this.evacuate();
			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);

		} else if (this.state == PeopleState.NORMAL) {
			if (MathUtils.randomFloat(0.0f, 1.0f) > 0.99)
				this.randomMove();

			this.m_queryCallback.SetBody(m_body);
			AABB peopleAABB = new AABB(m_body.getPosition().sub(
					new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)), m_body
					.getPosition().add(
							new Vec2(PEOPLE_SAFE_LENGTH, PEOPLE_SAFE_WIDTH)));
			m_world.queryAABB(m_queryCallback, peopleAABB);
		} else if (this.state == PeopleState.DEAD) {

		}

		if (++m_stepCount % SAMPLE_PRECISION == 0) {
			this.addPath();
			m_stepCount = 0;
		}

	}

	public void evacuate() {
		float threshold = 0.1f;
		Vec2 pos = m_body.getPosition();
		float distance = MathUtils.distance(pos, targetVec);
		Vec2 speed = m_body.getLinearVelocity();
		m_body.setLinearVelocity(new Vec2());
		if (distance > threshold
				&& MathUtils.distance(speed, new Vec2()) < PEOPLE_RUN_SPEED) {
			speed = new Vec2((targetVec.x - pos.x) / distance
					* PEOPLE_RUN_SPEED, (targetVec.y - pos.y) / distance
					* PEOPLE_RUN_SPEED);
		}

		float damping = 1.0f;
		if (slowDownBeforeObstacle) {
			damping = getDampingBeforeObstacles();
		}

		if (flockBehave) {

		}

		speed = speed.mulLocal(damping);
		float angle = this.getAngleFromVec(speed);
		m_body.setLinearVelocity(speed);
		m_body.setTransform(pos, angle);
	}
}

class WayPointComparer implements Comparator<WayPoint> {
	private WayPoint current = null;

	public void init(WayPoint current) {
		this.current = current;
	}

	@Override
	public int compare(WayPoint o1, WayPoint o2) {
		/*
		 * if(current.Weigh == 5 && o1.Weigh !=8){ return 1; }else
		 * if(current.Weigh == 5 && o2.Weigh !=8){ return -1; }
		 */
		/*
		 * if(o1.Weigh - current.Weigh >5 && current.Weigh>0 ) return 1; //防止跃迁
		 * else if(o2.Weigh - current.Weigh >5 && current.Weigh>0) return -1;
		 */
		int dist1 = (int) MathUtils.distanceSquared(current.Vec, o1.Vec);
		int dist2 = (int) MathUtils.distanceSquared(current.Vec, o2.Vec);
		if (Math.abs(dist1 - dist2) < 5.0f)
			if (o2.Weigh - o1.Weigh == 0) {
				return o1.ChooseCount - o2.ChooseCount;
			} else
				return o2.Weigh - o1.Weigh;
		else
			return dist1 - dist2;
	}
}

class JQueryCallback implements QueryCallback {
	private Body m_body;
	public Vec2 subVelocity;

	public void SetBody(Body body) {
		this.m_body = body;
	}

	public boolean reportFixture(Fixture fixture) {
		subVelocity = new Vec2();
		while (fixture != null) {
			if (fixture.getBody() != m_body
					&& fixture.getBody().getPosition().length() > 0.001f) {
				Vec2 tmp = m_body.getPosition().sub(
						fixture.getBody().getPosition());
				subVelocity.addLocal(tmp.mul(1.0f / tmp.length()));
			}
			fixture = fixture.getNext();
		}

		Vec2 velocity = m_body.getLinearVelocity();
		velocity.addLocal(subVelocity);
		if (velocity.length() > 5.0)
			velocity.mulLocal(5.0f / velocity.length());
		m_body.setLinearVelocity(velocity);
		return true;
	}
}

class JSmokeQueryCallback implements QueryCallback {
	private Body s_body;

	public void SetBody(Body body) {
		this.s_body = body;
	}

	public boolean reportFixture(Fixture fixture) {
		while (fixture != null) {
			if (fixture.getBody() != s_body
					&& !(fixture.getBody().getUserData() instanceof SmokeAgent)
					&& fixture.getBody().getPosition().length() > 0.001f) {

				Random rand = new Random(Calendar.getInstance()
						.getTimeInMillis());
				float f = rand.nextFloat();
				if (f > 0.8) {
					// s_body.setLinearVelocity(s_body.getLinearVelocity().negateLocal());

				} else {

					((SmokeAgent) s_body.getUserData()).state = SmokeState.STATIC;
				}
				// s_body.setLinearVelocity(new Vec2(-1f,-1f));
			}
			fixture = fixture.getNext();
		}

		return true;
	}
}

class JSimpleQueryCallback implements QueryCallback {
	Body m_self;
	boolean hasObj = false;

	public void init(Body body) {
		this.m_self = body;
		hasObj = false;
	}

	public boolean reportFixture(Fixture fixture) {

		while (fixture != null) {
			if (fixture.getBody() != m_self) {
				hasObj = true;
				return true;
			}
			fixture = fixture.getNext();
		}
		return true;
	}
}

// hook contact
class JContactListener implements ContactListener {

	@Override
	public void beginContact(Contact contact) {

	}

	@Override
	public void endContact(Contact contact) {
		// TODO Auto-generated method stub

	}

	@Override
	public void preSolve(Contact contact, Manifold oldManifold) {
		// TODO Auto-generated method stub

	}

	@Override
	public void postSolve(Contact contact, ContactImpulse impulse) {
		// TODO Auto-generated method stub

	}

}

class WayPoint {
	public Vec2 Vec;
	public int Weigh;
	public int ChooseCount;
	public int floor;

	public WayPoint(Vec2 vec, int weigh, int floor) {
		this.Vec = vec;
		this.Weigh = weigh;
		this.ChooseCount = 0;
		this.floor = floor;
	}

	public WayPoint(Vec2 vec, int weigh) {
		this.Vec = vec;
		this.Weigh = weigh;
		this.ChooseCount = 0;
		this.floor = 0;
	}

	public WayPoint Clone() {
		WayPoint p = new WayPoint(Vec.clone(), Weigh);
		return p;
	}

}

class JRayCastClosestCallback implements RayCastCallback {
	boolean m_hit;
	Vec2 m_point;
	Vec2 m_normal;
	private Body m_body;

	public void init(Body body) {
		m_hit = false;
		m_body = body;
	}

	public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal,
			float fraction) {
		Body body = fixture.getBody();
		fixture = fixture.m_next;
		if (body.getUserData() instanceof PeopleAgent || body == m_body)
			return -1f;
		else
			m_hit = true;

		m_point = point;
		m_normal = normal;
		return fraction;
	}

};

class Incident {
	public Vec2 position = new Vec2();
	public float radius = 0.0f;
	public float time = 0.0f;

	public Incident(Vec2 position, float radius, float time) {
		this.position = position;
		this.radius = radius;
		this.time = time;
	}

	public Incident(Vec2 position) {
		this.position = position;
	}

	public Incident(float radius) {
		this.radius = radius;
	}

	public Incident() {
	}
}

class ModelFactory {
	public static PeopleAgent createPeople(String peopleType, int id,
			SimpleSquare scene) {
		if (peopleType.equals("man")) {
			return new ManAgent(id, scene);
		} else if (peopleType.equals("woman")) {
			return new WomanAgent(id, scene);
		} else if (peopleType.equals("elder")) {
			return new ElderAgent(id, scene);
		} else if (peopleType.equals("kid")) {
			return new KidAgent(id, scene);
		}
		return null;
	}
}
