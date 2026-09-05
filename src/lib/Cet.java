package lib;

//public class Cet {
//    private long v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12;
//    private long[] V;
//    public Cet(int problemSize) {
//        this.V = new long[getMaxModifier(problemSize)];
//    }
//    public int getMaxModifier(int size) {
//        return (int) Math.ceil(size / 63.);
//    }
//
//    public int i2multiplier(int elem){
//        return (int) Math.floor(elem / 63.);
//    }
//
//    public void add(int elem){
//        int multiplier = i2multiplier(elem);
//        int pos = elem - multiplier *63;
//        V[multiplier] = (V[multiplier] | (1L << pos));
//    }
//}

import java.util.ArrayList;
import java.util.List;

public class Cet {

    private static final int BITS_PER_LONG = 64; // A long has 64 bits
    public long[] V;

    public Cet(int problemSize) {
        // Calculate the number of longs needed to store `problemSize` bits
        this.V = new long[(problemSize + BITS_PER_LONG - 1) / BITS_PER_LONG];
    }

    /**
     * Adds an element to the set by setting the corresponding bit.
     */
    public void add(int elem) {
        if (elem < 0 || elem >= V.length * BITS_PER_LONG) {
            throw new IllegalArgumentException("Element out of range: " + elem);
        }
        int multiplier = elem / BITS_PER_LONG; // Which long to use
        int pos = elem % BITS_PER_LONG;       // Which bit in the long
        V[multiplier] |= (1L << pos);          // Set the bit
    }
    /**
     * Removes an element from the set by clearing the corresponding bit.
     */
    public void remove(int elem) {
        if (elem < 0 || elem >= V.length * BITS_PER_LONG) {
            throw new IllegalArgumentException("Element out of range: " + elem);
        }
        int multiplier = elem / BITS_PER_LONG; // Which long to use
        int pos = elem % BITS_PER_LONG;       // Which bit in the long
        V[multiplier] &= ~(1L << pos);        // Clear the bit
    }

    /**
     * Performs a union operation with another Cet instance.
     * Modifies the current instance to represent the union of the two sets.
     */
    public void union(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }
        for (int i = 0; i < V.length; i++) {
            this.V[i] |= other.V[i]; // Bitwise OR operation
        }
    }

    /**
     * Removes from this Cet all elements that are contained in the other Cet.
     * This operation modifies the current instance.
     */
    public void removeAll(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }
        for (int i = 0; i < V.length; i++) {
            this.V[i] &= ~other.V[i]; // Bitwise AND with the complement of other.V[i]
        }
    }

    /**
     * Performs an intersection operation with another Cet instance.
     * Returns a new long[] array representing the intersection of the two sets.
     */
    public long[] intersect(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }

        // Create a new array to store the intersection
        long[] intersection = new long[this.V.length];

        // Compute the intersection
        for (int i = 0; i < this.V.length; i++) {
            intersection[i] = this.V[i] & other.V[i]; // Bitwise AND operation
        }

        return intersection;
    }

    public Cet intersectCet(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }

        // Create a new array to store the intersection
        long[] intersection = new long[this.V.length];

        // Compute the intersection
        for (int i = 0; i < this.V.length; i++) {
            intersection[i] = this.V[i] & other.V[i]; // Bitwise AND operation
        }
        Cet cet = new Cet(V.length);
        cet.V = intersection;
        return cet;
    }

    public boolean intersects(Cet other){
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }
        for (int i = 0; i < this.V.length; i++) {
            long v = this.V[i] & other.V[i]; // Bitwise AND operation
            if(v != 0x00L)
                return true;
        }
        return false;
    }


    /**
     * Checks if an element is present in the set.
     */
    public boolean contains(int elem) {
        if (elem < 0 || elem >= V.length * BITS_PER_LONG) {
            return false;
        }
        int multiplier = elem / BITS_PER_LONG;
        int pos = elem % BITS_PER_LONG;
        return (V[multiplier] & (1L << pos)) != 0;
    }



    /**
     * Returns a list of all indices where bits are set.
     * Each index is globally unique, accounting for the position of the long in the array.
     */
    public List<Integer> getIndexList() {
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < V.length; i++) {
            long set = V[i];
            while (set != 0) {
                int id = Long.numberOfTrailingZeros(set);  // Find the position of the lowest set bit
                indices.add(i * BITS_PER_LONG + id);      // Add the global index
                set &= (set - 1);  // Turn off the lowest set bit
            }
        }
        return indices;
    }

    public long[] getXORIndices(Cet theirs, List<Integer> indices) {
        long[] xor = new long[V.length];
        for (int i = 0; i < xor.length; i++) {
            xor[i] = V[i] ^ theirs.V[i];
        }
        for (int i = 0; i < xor.length; i++) {
            long set = xor[i];
            while (set != 0) {
                int id = Long.numberOfTrailingZeros(set);  // Find the position of the lowest set bit
                indices.add(i * BITS_PER_LONG + id);      // Add the global index
                set &= (set - 1);  // Turn off the lowest set bit
            }
        }
        return xor;
    }

    public long[] getXOR(Cet theirs) {
        long[] xor = new long[V.length];
        for (int i = 0; i < xor.length; i++) {
            xor[i] = V[i] ^ theirs.V[i];
        }
        return xor;
    }

    public List<Integer> getIndexList(long[] longs) {
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < longs.length; i++) {
            long set = longs[i];
            while (set != 0) {
                int id = Long.numberOfTrailingZeros(set);  // Find the position of the lowest set bit
                indices.add(i * BITS_PER_LONG + id);      // Add the global index
                set &= (set - 1);  // Turn off the lowest set bit
            }
        }
        return indices;
    }

    /**
     * Returns a list of lists, where each inner list contains the indices of set bits
     * for the corresponding long in the V array.
     */
    public List<List<Integer>> getIndexListByLong() {
        List<List<Integer>> result = new ArrayList<>();
        for (int i = 0; i < V.length; i++) {
            List<Integer> indices = new ArrayList<>();
            long set = V[i];
            while (set != 0) {
                int id = Long.numberOfTrailingZeros(set);  // Find the position of the lowest set bit
                indices.add(id);                         // Add the local index
                set &= (set - 1);  // Turn off the lowest set bit
            }
            result.add(indices);
        }
        return result;
    }

    /**
     * Computes the set difference U_3 = U_1 \ U_2 (elements in this U_1 but not in other U_2).
     * Returns the result.
     */
    public long[] difference(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }
        long[] result = new long[(this.V.length * BITS_PER_LONG + BITS_PER_LONG - 1) / BITS_PER_LONG];
        for (int i = 0; i < V.length; i++) {
            result[i] = this.V[i] & ~other.V[i]; // Bitwise AND with the complement of other.V[i]
        }
        return result;
    }

    /**
     * Checks if this Cet is a subset of another Cet.
     * Returns true if all bits set in this.V are also set in other.V.
     */
    public boolean isSubset(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }

        for (int i = 0; i < V.length; i++) {
            // Check if all bits set in this.V[i] are also set in other.V[i]
            if ((this.V[i] & other.V[i]) != this.V[i]) {
                return false;
            }
        }
        return true;
    }

    public boolean isEmpty() {
        for (int i = 0; i < V.length; i++)
            if(V[i] != 0x00L)
                return false;
        return true;
    }
    public boolean equals(Cet other) {
        if (this.V.length != other.V.length) {
            throw new IllegalArgumentException("Both Cet instances must have the same problem size.");
        }

        for (int i = 0; i < V.length; i++) {
            if (this.V[i] != other.V[i]) {
                return false;
            }
        }
        return true;
    }

    public Cet copy(int elem){
        Cet copy = copy();
        copy.add(elem);
        return copy;
    }
    public Cet copy() {
        Cet copy = new Cet(this.V.length * BITS_PER_LONG);
        copy.union(this);
        return copy;
    }

}
