unit UFixedJoint;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TFixedJoint = class(TTester)
   public
      constructor Create; override;
   end;

implementation

{ TFixedJoint }

constructor TFixedJoint.Create;
var
   bd, bd1, bd2: Tb2BodyDef;
   sd, sd1, sd2: Tb2PolygonShape;
   ground, body1, body2: Tb2Body;
   jd: Tb2FixedJointDef;
   fd: Tb2FixtureDef;
   vertices: array[0..2] of TVector2;
begin
   inherited;
   begin
      bd := Tb2BodyDef.Create;
      {$IFDEF OP_OVERLOAD}
      bd.position.SetValue(0.0, -10.0);
      {$ELSE}
      SetValue(bd.position, 0.0, -10.0);
      {$ENDIF}
      ground := m_world.CreateBody(bd);

      sd := Tb2PolygonShape.Create;
      sd.SetAsBox(50.0, 10.0);
      ground.CreateFixture(sd, 0.0);
   end;

   begin
      bd1 := Tb2BodyDef.Create;
      bd1.bodyType := b2_dynamicBody;
      SetValue(bd1.position, 1.0, 7.0);
      bd1.angle := 0.7;
      body1 := m_world.CreateBody(bd1);

      sd1 := Tb2PolygonShape.Create;
      SetValue(vertices[0], -2.0, -2.0);
      SetValue(vertices[1], 2.0, -2.0);
      SetValue(vertices[2], 2.0, 2.0);
      sd1.SetVertices(@vertices[0], 3);

      fd := Tb2FixtureDef.Create;
      fd.shape := sd1;
      fd.density := 10.0;
      fd.friction := 0.2;
      body1.CreateFixture(fd);

      bd2 := Tb2BodyDef.Create;
      bd2.bodyType := b2_dynamicBody;
      SetValue(bd2.position, 5.0, 6.0);
      bd2.angle := -0.2;
      body2 := m_world.CreateBody(bd2);

      sd2 := Tb2PolygonShape.Create;
      sd2.SetAsBox(1.0, 2.0);
      fd := Tb2FixtureDef.Create;
      fd.shape := sd2;
      fd.density := 30.0;
      fd.friction := 0.2;
      body2.CreateFixture(fd);

      jd := Tb2FixedJointDef.Create;
      jd.collideConnected := False;
      jd.Initialize(body1, body2);
      m_world.CreateJoint(jd);
   end;

   begin
      bd1 := Tb2BodyDef.Create;
      bd1.bodyType := b2_dynamicBody;
      SetValue(bd1.position, 20, 20);
      body1 := m_world.CreateBody(bd1);

      sd1 := Tb2PolygonShape.Create;
      sd1.SetAsBox(2, 2);
      fd := Tb2FixtureDef.Create;
      fd.shape := sd1;
      fd.density := 10.0;
      fd.friction := 0.5;
      fd.restitution := 0.0;
      body1.CreateFixture(fd);

      bd2 := Tb2BodyDef.Create;
      bd2.bodyType := b2_dynamicBody;
      SetValue(bd2.position, -5, 10);
      body2 := m_world.CreateBody(bd2);

      sd2 := Tb2PolygonShape.Create;
      sd2.SetAsBox(2.0, 2.0);
      fd := Tb2FixtureDef.Create;
      fd.shape := sd2;
      fd.density := 10.0;
      fd.friction := 0.5;
      fd.restitution := 0.0;
      body2.CreateFixture(fd);

      jd := Tb2FixedJointDef.Create;
      jd.collideConnected := False;
      jd.Initialize(body1, body2);
      m_world.CreateJoint(jd);
   end;
end;

initialization
   RegisterTestEntry('Fixed Joint', TFixedJoint);

end.
