package edu.arizona.verbs.environments.ww2d;


import org.lwjgl.opengl.Display;
import org.newdawn.slick.AppGameContainer;
import org.newdawn.slick.BasicGame;
import org.newdawn.slick.GameContainer;
import org.newdawn.slick.Graphics;
import org.newdawn.slick.SlickException;
import org.newdawn.slick.opengl.renderer.SGL;

import edu.arizona.simulator.ww2d.system.GameSystem;

public class VerbGameContainer extends AppGameContainer {


	public VerbGameContainer(int width,int height) throws SlickException {
		super(new Visualization(), width, height, false);
	}

	/**
	 * Start running the game
	 * 
	 * @throws SlickException Indicates a failure to initialise the system
	 */
	public void start() {
		try {
			setup();
		} catch (SlickException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Render one frame for the system.
	 * @param system
	 */
	public void render(GameSystem system) { 
		if (clearEachFrame) {
			GL.glClear(SGL.GL_COLOR_BUFFER_BIT | SGL.GL_DEPTH_BUFFER_BIT);
		} 
		
		GL.glLoadIdentity();
		
		Graphics graphics = getGraphics();
		graphics.resetFont();
		graphics.resetLineWidth();
		graphics.setAntiAlias(false);
		
		system.render(graphics);

		graphics.resetTransform();
		
		GL.flush();
		Display.update();
	}
}

class Visualization extends BasicGame {

	public Visualization() { 
		super("Verb Learning");
	}
	
	@Override
	public void render(GameContainer arg0, Graphics arg1) throws SlickException {}

	@Override
	public void init(GameContainer arg0) throws SlickException {}

	@Override
	public void update(GameContainer arg0, int arg1) throws SlickException {} 
	
}