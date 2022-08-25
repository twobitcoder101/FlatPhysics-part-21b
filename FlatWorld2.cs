using System;
using System.Collections.Generic;

namespace FlatPhysics
{
    public enum BroadPhase
    {
        BruteForce, Grid
    }

    public sealed class FlatWorld2
    {
        public static int TransformCount = 0;
        public static int NoTransformCount = 0;

        public static readonly float MinBodySize = 0.01f * 0.01f;
        public static readonly float MaxBodySize = 64f * 64f;

        public static readonly float MinDensity = 0.5f;     // g/cm^3
        public static readonly float MaxDensity = 21.4f;

        public static readonly int MinIterations = 1;
        public static readonly int MaxIterations = 128;

        private FlatVector gravity;
        private List<FlatBody> bodyList;
        private List<FlatManifold> contactList;

        public List<FlatVector> ContactPointsList;

        private Dictionary<int, List<int>> grid;
        private BroadPhase broadPhase;
        
        private float gridNodeSize;

        public static float MinNodeSize = 1;
        public static float MaxNodeSize = 32;


        public int BodyCount
        {
            get { return this.bodyList.Count; }
        }

        public FlatWorld2()
        {
            this.gravity = new FlatVector(0f, -9.81f);
            this.bodyList = new List<FlatBody>();
            this.contactList = new List<FlatManifold>();


            this.ContactPointsList = new List<FlatVector>();

            this.grid = new Dictionary<int, List<int>>();
            this.broadPhase = BroadPhase.Grid;
            this.gridNodeSize = 4;
        }

        public void AddBody(FlatBody body)
        {
            this.bodyList.Add(body);
        }

        public bool RemoveBody(FlatBody body)
        {
            return this.bodyList.Remove(body);
        }

        public bool GetBody(int index, out FlatBody body)
        {
            body = null;

            if (index < 0 || index >= this.bodyList.Count)
            {
                return false;
            }

            body = this.bodyList[index];
            return true;
        }

        public void Step(float time, int iterations)
        {
            iterations = FlatMath.Clamp(iterations, FlatWorld.MinIterations, FlatWorld.MaxIterations);

            this.ContactPointsList.Clear();

            int columns = 0;

            if(this.broadPhase is BroadPhase.Grid)
            {
                this.BuildNodeGrid(out columns);
            }

            for (int it = 0; it < iterations; it++)
            {
                // Movement step
                for (int i = 0; i < this.bodyList.Count; i++)
                {
                    this.bodyList[i].Step(time, this.gravity, iterations);
                }

                if(this.broadPhase is BroadPhase.BruteForce)
                {
                    this.CollisionStepBruteForce();
                }
                else if(this.broadPhase is BroadPhase.Grid)
                {
                    this.CollisionStepGrid(columns);
                }

            }
        }

        private void CollisionStepBruteForce()
        {
            this.contactList.Clear();
            this.grid.Clear();

            // collision step
            for (int i = 0; i < this.bodyList.Count - 1; i++)
            {
                FlatBody bodyA = this.bodyList[i];
                FlatAABB bodyA_aabb = bodyA.GetAABB();

                for (int j = i + 1; j < this.bodyList.Count; j++)
                {
                    FlatBody bodyB = this.bodyList[j];
                    FlatAABB bodyB_aabb = bodyB.GetAABB();

                    if (bodyA.IsStatic && bodyB.IsStatic)
                    {
                        continue;
                    }

                    if (!Collisions.IntersectAABBs(bodyA_aabb, bodyB_aabb))
                    {
                        continue;
                    }

                    if (Collisions.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
                    {
                        if (bodyA.IsStatic)
                        {
                            bodyB.Move(normal * depth);
                        }
                        else if (bodyB.IsStatic)
                        {
                            bodyA.Move(-normal * depth);
                        }
                        else
                        {
                            bodyA.Move(-normal * depth / 2f);
                            bodyB.Move(normal * depth / 2f);
                        }

                        Collisions.FindContactPoints(bodyA, bodyB, out FlatVector contact1, out FlatVector contact2, out int contactCount);
                        FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                        this.contactList.Add(contact);
                    }
                }
            }

            for (int i = 0; i < this.contactList.Count; i++)
            {
                FlatManifold contact = this.contactList[i];
                this.ResolveCollision(in contact);

                if (contact.ContactCount > 0)
                {
                    this.ContactPointsList.Add(contact.Contact1);

                    if (contact.ContactCount > 1)
                    {
                        this.ContactPointsList.Add(contact.Contact2);
                    }
                }
            }
        }

        private void BuildNodeGrid(/*out float minX, out float minY, out float maxX, out float maxY,*/ out int columns /*, out int rows*/)
        {
            float minX = float.MaxValue;
            //float minY = float.MaxValue;
            float maxX = float.MinValue;
            //float maxY = float.MaxValue;

            for(int i = 0; i < this.bodyList.Count; i++)
            {
                FlatBody body = this.bodyList[i];
                FlatAABB aabb = body.GetAABB();

                if(aabb.Min.X < minX) { minX = aabb.Min.X; }
                if(aabb.Max.X > maxX) { maxX = aabb.Max.X; }
                //if(aabb.Min.Y < minY) { minY = aabb.Min.Y; }
                //if(aabb.Max.Y > maxY) { maxY = aabb.Max.Y; }
            }

            float width = maxX - minX;
            //float height = maxY - minY;

            columns = (int)MathF.Ceiling(width / this.gridNodeSize);
            //rows = (int)MathF.Ceiling(height / this.gridNodeSize);
        }

        private void CollisionStepGrid(int columns)
        {
            this.contactList.Clear();

            foreach(KeyValuePair<int, List<int>> pair in this.grid)
            {
                pair.Value.Clear();
            }

            for(int i = 0; i < this.bodyList.Count; i++)
            {
                FlatBody body = this.bodyList[i];
                FlatAABB aabb = body.GetAABB();

                int left = (int)MathF.Floor(aabb.Min.X / this.gridNodeSize);
                int right = (int)MathF.Ceiling(aabb.Max.X / this.gridNodeSize);
                int bottom = (int)MathF.Floor(aabb.Min.Y / this.gridNodeSize);
                int top = (int)MathF.Ceiling(aabb.Max.Y / this.gridNodeSize);

                for(int y = bottom; y <= top; y++)
                {
                    for(int x = left; x <= right; x++)
                    {
                        int key = x + y * columns;

                        if(!this.grid.TryGetValue(key, out List<int> node))
                        {
                            node = new List<int>();
                            this.grid.Add(key, node);
                        }

                        if (!node.Contains(i))
                        {
                            node.Add(i);
                        }
                    }
                }
            }

            foreach(KeyValuePair<int, List<int>> pair in this.grid)
            {
                List<int> node = pair.Value;

                for (int i = 0; i < node.Count - 1; i++)
                {
                    FlatBody bodyA = this.bodyList[node[i]];
                    FlatAABB bodyA_aabb = bodyA.GetAABB();

                    for (int j = i + 1; j < node.Count; j++)
                    {
                        FlatBody bodyB = this.bodyList[node[j]];
                        FlatAABB bodyB_aabb = bodyB.GetAABB();

                        if (bodyA.IsStatic && bodyB.IsStatic)
                        {
                            continue;
                        }

                        if (!Collisions.IntersectAABBs(bodyA_aabb, bodyB_aabb))
                        {
                            continue;
                        }

                        if(this.ContactListContainsPair(bodyA, bodyB))
                        {
                            continue;
                        }

                        if (Collisions.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
                        {
                            if (bodyA.IsStatic)
                            {
                                bodyB.Move(normal * depth);
                            }
                            else if (bodyB.IsStatic)
                            {
                                bodyA.Move(-normal * depth);
                            }
                            else
                            {
                                bodyA.Move(-normal * depth / 2f);
                                bodyB.Move(normal * depth / 2f);
                            }

                            Collisions.FindContactPoints(bodyA, bodyB, out FlatVector contact1, out FlatVector contact2, out int contactCount);
                            FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                            this.contactList.Add(contact);
                        }
                    }
                }
            }

            for (int i = 0; i < this.contactList.Count; i++)
            {
                FlatManifold contact = this.contactList[i];
                this.ResolveCollision(in contact);

                if (contact.ContactCount > 0)
                {
                    this.ContactPointsList.Add(contact.Contact1);

                    if (contact.ContactCount > 1)
                    {
                        this.ContactPointsList.Add(contact.Contact2);
                    }
                }
            }
        }

        private bool ContactListContainsPair(FlatBody bodyA, FlatBody bodyB)
        {
            for(int i = 0; i < this.contactList.Count; i++)
            {
                FlatManifold manifold = this.contactList[i];

                if((manifold.BodyA == bodyA && manifold.BodyB == bodyB) ||
                    (manifold.BodyB == bodyA && manifold.BodyA == bodyB))
                {
                    return true;
                }
            }

            return false;
        }

        public void ResolveCollision(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            float depth = contact.Depth;

            FlatVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (FlatMath.Dot(relativeVelocity, normal) > 0f)
            {
                return;
            }

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float j = -(1f + e) * FlatMath.Dot(relativeVelocity, normal);
            j /= bodyA.InvMass + bodyB.InvMass;

            FlatVector impulse = j * normal;

            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyB.LinearVelocity += impulse * bodyB.InvMass;
        }


    }
}

