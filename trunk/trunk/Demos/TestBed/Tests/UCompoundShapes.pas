unit UCompoundShapes;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TCompoundShapes = class(TTester)
   public
      constructor Create; override;
   end;

implementation

{ TCompoundShapes }

constructor TCompoundShapes.Create;
var
   i: Integer;
   bd: Tb2BodyDef;
   body: Tb2Body;
   shape, polygon1, polygon2, triangle1, triangle2, bottom, left, right: Tb2PolygonShape;
   circle1, circle2: Tb2CircleShape;
   xf1, xf2: Tb2Transform;
   vertices: array[0..2] of TVector2;
begin
   inherited;
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, 0.0, 0.0);
      body := m_world.CreateBody(bd);

      shape := Tb2PolygonShape.Create;
      shape.SetAsEdge(MakeVector(50.0, 0.0), MakeVector(-50.0, 0.0));
      body.CreateFixture(shape, 0.0);
   end;

   begin
      circle1 := Tb2CircleShape.Create;
      circle1.m_radius := 0.5;
      SetValue(circle1.m_p, -0.5, 0.5);

      circle2 := Tb2CircleShape.Create;
      circle2.m_radius := 0.5;
      SetValue(circle2.m_p, 0.5, 0.5);

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      for i := 0 to 9 do
      begin
         SetValue(bd.position, RandomFloat(-0.1, 0.1) + 5.0, 1.05 + 2.5 * i);
         bd.angle := RandomFloat(-Pi, Pi);
         body := m_world.CreateBody(bd, False);
         body.CreateFixture(circle1, 2.0, False, False);
         body.CreateFixture(circle2, 0.0, False);
      end;
      bd.Free;
      circle1.Free;
      circle2.Free;
   end;

   begin
      polygon1 := Tb2PolygonShape.Create;
      polygon1.SetAsBox(0.25, 0.5);

      polygon2 := Tb2PolygonShape.Create;
      polygon2.SetAsBox(0.25, 0.5, MakeVector(0.0, -0.5), 0.5 * Pi);

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      for i := 0 to 9 do
      begin
         SetValue(bd.position, RandomFloat(-0.1, 0.1) - 5.0, 1.05 + 2.5 * i);
         bd.angle := RandomFloat(-Pi, Pi);
         body := m_world.CreateBody(bd, False);
         body.CreateFixture(polygon1, 2.0, False, False);
         body.CreateFixture(polygon2, 2.0, False);
      end;
      bd.Free;
      polygon1.Free;
      polygon2.Free;
   end;

   begin
      {$IFDEF OP_OVERLOAD}
      xf1.R.SetValue(0.3524 * Pi);
      {$ELSE}
      SetValue(xf1.R, 0.3524 * Pi);
      {$ENDIF}
      xf1.position := b2Mul(xf1.R, MakeVector(1.0, 0.0));

      triangle1 := Tb2PolygonShape.Create;
      vertices[0] := b2Mul(xf1, MakeVector(-1.0, 0.0));
      vertices[1] := b2Mul(xf1, MakeVector(1.0, 0.0));
      vertices[2] := b2Mul(xf1, MakeVector(0.0, 0.5));
      triangle1.SetVertices(@vertices[0], 3);

      {$IFDEF OP_OVERLOAD}
      xf2.R.SetValue(-0.3524 * Pi);
      {$ELSE}
      SetValue(xf2.R, -0.3524 * Pi);
      {$ENDIF}
      xf2.position := b2Mul(xf2.R, MakeVector(-1.0, 0.0));

      triangle2 := Tb2PolygonShape.Create;
      vertices[0] := b2Mul(xf2, MakeVector(-1.0, 0.0));
      vertices[1] := b2Mul(xf2, MakeVector(1.0, 0.0));
      vertices[2] := b2Mul(xf2, MakeVector(0.0, 0.5));
      triangle2.SetVertices(@vertices[0], 3);

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      for i := 0 to 9 do
      begin
         SetValue(bd.position, RandomFloat(-0.1, 0.1), 2.05 + 2.5 * i);
         bd.angle := 0.0;
         body := m_world.CreateBody(bd, False);
         body.CreateFixture(triangle1, 2.0, False, False);
         body.CreateFixture(triangle2, 2.0, False);
      end;
      triangle1.Free;
      triangle2.Free;
      bd.Free;
   end;

   begin
      bottom := Tb2PolygonShape.Create;
      bottom.SetAsBox( 1.5, 0.15 );

      left := Tb2PolygonShape.Create;
      left.SetAsBox(0.15, 2.7, MakeVector(-1.45, 2.35), 0.2);

      right := Tb2PolygonShape.Create;
      right.SetAsBox(0.15, 2.7, MakeVector(1.45, 2.35), -0.2);

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      SetValue(bd.position, 0.0, 2.0);
      body := m_world.CreateBody(bd);
      body.CreateFixture(bottom, 4.0, True, False);
      body.CreateFixture(left, 4.0, True, False);
      body.CreateFixture(right, 4.0);
   end;
end;

initialization
   RegisterTestEntry('Compound Shapes', TCompoundShapes);
end.
