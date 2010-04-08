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
   fd: Tb2FixtureDef;
   d, angle, delta: Float;
   cshape: Tb2CircleShape;
   vertices: array[0..5] of TVector2;
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
      shape.m_radius := 0.0;
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

   // Square made from edges notice how the edges are shrunk to account
   // for the polygon radius. This makes it so the square character does
   // not get snagged. However, ray casts can now go through the cracks.
   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

      shape := Tb2PolygonShape.Create;
      d := 2.0 * b2_polygonRadius;
      shape.SetAsEdge(MakeVector(-1.0 + d, 3.0), MakeVector(1.0 - d, 3.0));
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsEdge(MakeVector(1.0, 3.0 + d), MakeVector(1.0, 5.0 - d));
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsEdge(MakeVector(1.0 - d, 5.0), MakeVector(-1.0 + d, 5.0));
      ground.CreateFixture(shape, 0.0, False, False);
      shape.SetAsEdge(MakeVector(-1.0, 5.0 - d), MakeVector(-1.0, 3.0 + d));
      ground.CreateFixture(shape, 0.0);
   end;

   // Square character
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, -3.0, 5.0);
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

   // Hexagon character
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, -5.0, 5.0);
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
end;

initialization
   RegisterTestEntry('Character Collision', TCharacterCollision);
end.
