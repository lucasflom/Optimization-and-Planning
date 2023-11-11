public class IKChain {
    public float l, a;
    public Vec2 end;

    public IKChain(float l, float a) {
        this.l = l;
        this.a = a;
    }

    public void fk(Vec2 root) {
        end = new Vec2(cos(a)*l,sin(a)*l).plus(root);
    }

    public void fk(IKChain[] links) {
        float angle = 0;
        for (int i = 0; i < links.length; i++) {
            angle += links[i].a;
        }
        end = new Vec2(cos(angle)*l, sin(angle)*l).plus(links[links.length].end);
    }
}