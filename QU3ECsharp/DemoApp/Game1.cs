using System.Diagnostics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Qu3ECSharp.Collision;
using Qu3ECSharp.Dynamics;
using Qu3ECSharp.Math;
using Qu3ECSharp.Scene;

namespace DemoApp
{
    /// <summary>
    /// This is the main type for your game.
    /// </summary>
    public class Game1 : Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        private Scene scene = null;

        private Body body = null;
        private Body body2 = null;
        protected override void Initialize()
        {
            // TODO: Add your initialization logic here
            scene = new Scene(1.0f / 60.0f);

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);

            
            BodyDefinition bodyDefinition = new BodyDefinition();
            bodyDefinition.BodyType = BodyType.DynamicBody;
            bodyDefinition.Position = new Qu3ECSharp.Math.Vector3(0,5,0);
            
            body = scene.CreateBody(bodyDefinition);

            BoxDefinition boxDefinition = new BoxDefinition();
            Transform localTransform = new Transform();
            Transform.Identity(localTransform);
            boxDefinition.Set(localTransform,new Qu3ECSharp.Math.Vector3(1.0f,1.0f,1.0f));
            body.AddBox(boxDefinition);



            bodyDefinition = new BodyDefinition();
            bodyDefinition.BodyType = BodyType.StaticBody;
            body2 = scene.CreateBody(bodyDefinition);
            boxDefinition = new BoxDefinition();
            localTransform = new Transform();
            Transform.Identity(localTransform);
            boxDefinition.Set(localTransform,new Qu3ECSharp.Math.Vector3(10.0f,1.0f,10.0f));
            body2.AddBox(boxDefinition);

            // TODO: use this.Content to load your game content here
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// game-specific content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            // TODO: Add your update logic here

            scene.Step();
            this.Window.Title = body.Transform.Position.Y.ToString() + " " + body2.Transform.Position.Y.ToString();

            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            // TODO: Add your drawing code here

            base.Draw(gameTime);
        }
    }
}
