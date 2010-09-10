unit UCharacterCollision;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TCharacterCollision = class(TTester)
   public
      constructor Create; override;
      procedure Step(var settings: TSettings; timeStep: Float); override;
   end;


implementation

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.

{ TCharacterCollision }

constructor TCharacterCollision.Create;
var
   i: Integer;
   bd: Tb2BodyDef;
   ground, body: Tb2Body;
   shape: Tb2PolygonShape;
   lshape: Tb2LoopShape;
   fd: Tb2FixtureDef;
   d, angle, delta: Float;
   cshape: Tb2CircleShape;
   vertices: array[0..5] of TVector2;
   vs: array[0..9] of TVector2;
begin
   inherited;
   // Ground body
   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

      shape := Tb2PolygonShape.Create;
      shape.SetAsEdge(MakeVector(-20.0, 0.0), MakeVector(20.0, 0.0));
      ground.CreateFixture(shape, 0.0);
   end;

   // Collinear edges
   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

      shape := Tb2PolygonShape.Create;
      shape.SetAsEdge(MakeVector(-8.0, 1.0), MakeVector(-6.0, 1.0));
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsEdge(MakeVector(-6.0, 1.0), MakeVector(-4.0, 1.0));
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsEdge(MakeVector(-4.0, 1.0), MakeVector(-2.0, 1.0));
      ground.CreateFixture(shape, 0.0);
   end;

   // Square tiles
   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

      shape := Tb2PolygonShape.Create;
      shape.SetAsBox(1.0, 1.0, MakeVector(4.0, 3.0), 0.0);
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsBox(1.0, 1.0, MakeVector(6.0, 3.0), 0.0);
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsBox(1.0, 1.0, MakeVector(8.0, 3.0), 0.0);
      ground.CreateFixture(shape, 0.0);
   end;

   // Square made from an edge loop.
   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

			SetValue(vertices[0], -1.0, 3.0);
			SetValue(vertices[1], 1.0, 3.0);
			SetValue(vertices[2], 1.0, 5.0);
			SetValue(vertices[3], -1.0, 5.0);
			lshape := Tb2LoopShape.Create;
      lshape.SetVertices(@vertices[0], 4);
			ground.CreateFixture(lshape, 0.0);
   end;

   // Edge loop
   begin
			bd := Tb2BodyDef.Create;
			SetValue(bd.position, -10.0, 4.0);
			ground := m_world.CreateBody(bd);

			SetValue(vs[0], 0.0, 0.0);
			SetValue(vs[1], 6.0, 0.0);
			SetValue(vs[2], 6.0, 2.0);
			SetValue(vs[3], 4.0, 1.0);
			SetValue(vs[4], 2.0, 2.0);
			SetValue(vs[5], 0.0, 2.0);
			SetValue(vs[6], -2.0, 2.0);
			SetValue(vs[7], -4.0, 3.0);
			SetValue(vs[8], -6.0, 2.0);
			SetValue(vs[9], -6.0, 0.0);
			lshape := Tb2LoopShape.Create;
      lshape.SetVertices(@vs[0], 10);
			ground.CreateFixture(lshape, 0.0);
   end;

   // Square character 1
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, -3.0, 8.0);
      bd.bodyType := b2_dynamicBody;
      bd.fixedRotation := True;
      bd.allowSleep := False;

      body := m_world.CreateBody(bd);
      shape := Tb2PolygonShape.Create;
      shape.SetAsBox(0.5, 0.5);

      fd := Tb2FixtureDef.Create;
      fd.shape := shape;
      fd.density := 20.0;
      body.CreateFixture(fd);
   end;

	 // Square character 2
   begin
			bd := Tb2BodyDef.Create;
			SetValue(bd.position, -5.0, 5.0);
			bd.bodyType := b2_dynamicBody;
			bd.fixedRotation := True;
			bd.allowSleep := False;

			body := m_world.CreateBody(bd);

			shape := Tb2PolygonShape.Create;
			shape.SetAsBox(0.25, 0.25);

			fd := Tb2FixtureDef.Create;
			fd.shape := shape;
			fd.density := 20.0;
			body.CreateFixture(fd);
   end;

   // Hexagon character
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, -5.0, 8.0);
      bd.bodyType := b2_dynamicBody;
      bd.fixedRotation := True;
      bd.allowSleep := False;

      body := m_world.CreateBody(bd);

      angle := 0.0;
      delta := Pi / 3.0;
      for i := 0 to 5 do
      begin
         SetValue(vertices[i], 0.5 * Cos(angle), 0.5 * Sin(angle));
         angle := angle + delta;
      end;

      shape := Tb2PolygonShape.Create;
      shape.SetVertices(@vertices[0], 6);

      fd := Tb2FixtureDef.Create;
      fd.shape := shape;
      fd.density := 20.0;
      body.CreateFixture(fd);
   end;

   // Circle character
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, 3.0, 5.0);
      bd.bodyType := b2_dynamicBody;
      bd.fixedRotation := True;
      bd.allowSleep := False;

      body := m_world.CreateBody(bd);

      cshape := Tb2CircleShape.Create;
      cshape.m_radius := 0.5;

      fd := Tb2FixtureDef.Create;
      fd.shape := cshape;
      fd.density := 20.0;
      body.CreateFixture(fd);
   end;
end;

procedure TCharacterCollision.Step(var settings: TSettings; timeStep: Float);
begin
   inherited;
   DrawText('This demo tests various character collision shapes.');
   DrawText('Limitation: square and hexagon can snag on aligned boxes.');
   DrawText('Feature: loops have smooth collision inside and out.');
end;

initialization
   RegisterTestEntry('Character Collision', TCharacterCollision);
end.
