/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.indoor;

import java.io.Serializable;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.UUID;
import java.util.regex.Pattern;

/**
 * Encapsulates a beacon identifier of arbitrary byte length.
 * It can encapsulate an identifier that is a 16-byte UUID, or an integer.
 * Based on:
 * <a href="https://github.com/AltBeacon/android-beacon-library/blob/master/src/main/java/org/altbeacon/beacon/Identifier.java">
 *   https://github.com/AltBeacon/android-beacon-library/blob/master/src/main/java/org/altbeacon/beacon/Identifier.java
 * </a>
 */
public class BeaconIdentifier implements Comparable<BeaconIdentifier>, Serializable {
    /**
     * Parses beacon identifiers in hexadecimal format.
     */
    private static final Pattern HEX_PATTERN = Pattern.compile("^0x[0-9A-Fa-f]*$");

    /**
     * Parses beacon identifiers in hexadecimal format without prefix.
     */
    private static final Pattern HEX_PATTERN_NO_PREFIX = Pattern.compile("^[0-9A-Fa-f]*$");

    /**
     * Parses beacon identifiers in decimal format.
     */
    private static final Pattern DECIMAL_PATTERN = Pattern.compile("^(0|[1-9][0-9]*)$");

    /**
     * Parses beacon identifiers in UUID format.
     */
    private static final Pattern UUID_PATTERN = Pattern.compile(
            "^[0-9A-Fa-f]{8}-?[0-9A-Fa-f]{4}-?[0-9A-Fa-f]{4}-?[0-9A-Fa-f]{4}-?[0-9A-Fa-f]{12}$");

    /**
     * Maximum allowed identifier value from an integer.
     */
    private static final int MAX_INTEGER = 65535;

    /**
     * Contains digits to represent this instance in hexadecimal format.
     */
    private static final char[] HEX_DIGITS =
            {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

    /**
     * Internal value holding a beacon identifier as a byte array.
     */
    private byte[] value;


    /**
     * Empty constructor to prevent deserialization issues.
     */
    protected BeaconIdentifier() {
    }

    /**
     * Creates a nw instance of a beacon identifier.
     *
     * @param value value to use.
     * @throws NullPointerException if provided value is null.
     */
    protected BeaconIdentifier(final byte[] value) {
        if (value == null) {
            throw new NullPointerException(
                    "Identifiers cannot be constructed from null pointers but \"value\" is null.");
        }

        this.value = value;
    }

    /**
     * Takes the passed string and tries to figure out what format it is in.
     * Then turns the string into plain bytes and constructs an identifier.
     * <p>
     * This method parses UUIDs without dashes for compatibility (although this is not a standard behaviour).
     * <p>
     * Allowed formats:
     * <ul>
     *   <li>UUID: 2F234454-CF6D-4A0F-ADF2-F4911BA9FFA6 (16 bytes)</li>
     *   <li>Hexadecimal: 0x000000000003 (variable length)</li>
     *   <li>Decimal: 1337 (2 bytes)</li>
     * </ul>
     *
     * @param stringValue string to be parsed.
     * @return an identifier representing the specified value.
     * @throws NullPointerException     if string value is null.
     * @throws IllegalArgumentException if parsing fails for some other reason (invalid format, etc.).
     * @see <a href="https://www.ietf.org/rfc/rfc4122.txt">RFC 4122 on UUIDs</a>
     */
    public static BeaconIdentifier parse(final String stringValue) {
        return parse(stringValue, -1);
    }

    /**
     * Variant of the parse method that allows specifying the byte length of the identifier.
     *
     * @param stringValue       value to be parsed.
     * @param desiredByteLength requested number of bytes to hold the identifier or -1 if not specified.
     * @return the parsed identifier.
     * @throws NullPointerException     if string value is null.
     * @throws IllegalArgumentException if parsing fails for some other reason (invalid format, etc.).
     */
    public static BeaconIdentifier parse(final String stringValue, final int desiredByteLength) {
        if (stringValue == null) {
            throw new NullPointerException(
                    "Identifiers cannot be constructed from null pointers but \"stringValue\" is null.");
        }

        if (HEX_PATTERN.matcher(stringValue).matches()) {
            // parse hexadecimal format
            return parseHex(stringValue.substring(2), desiredByteLength);
        }

        if (UUID_PATTERN.matcher(stringValue).matches()) {
            // parse UUID format
            return parseHex(stringValue.replace("-", ""), desiredByteLength);
        }

        if (DECIMAL_PATTERN.matcher(stringValue).matches()) {
            // parse decimal format
            var value = Integer.parseInt(stringValue);
            if (desiredByteLength <= 0 || desiredByteLength == 2) {
                return fromInt(value);
            } else {
                return fromLong(value, desiredByteLength);
            }
        }

        if (HEX_PATTERN_NO_PREFIX.matcher(stringValue).matches()) {
            // parse hexadecimal format without prefix
            return parseHex(stringValue, desiredByteLength);
        }

        throw new IllegalArgumentException("Unable to parse identifier");
    }

    /**
     * Creates an identifier backed by an array of length desiredByteLength.
     *
     * @param longValue         a long to put into the identifier.
     * @param desiredByteLength how many bytes to make the identifier.
     * @return the parsed identifier.
     * @throws IllegalArgumentException if desired number of bytes is negative.
     */
    public static BeaconIdentifier fromLong(long longValue, final int desiredByteLength) {
        if (desiredByteLength < 0) {
            throw new IllegalArgumentException("identifier length must be > 0");
        }
        final var newValue = new byte[desiredByteLength];
        for (var i = desiredByteLength - 1; i >= 0; i--) {
            newValue[i] = (byte) (longValue & 0xff);
            longValue = longValue >> 8;
        }
        return new BeaconIdentifier(newValue);
    }

    /**
     * Creates an identifier backed by a two byte array (big endian).
     *
     * @param intValue an integer between 0 and 65535 (inclusive).
     * @return an identifier with the specified value.
     * @throws IllegalArgumentException if provided value is out of valid range (from 0 to 65535).
     */
    public static BeaconIdentifier fromInt(final int intValue) {
        if (intValue < 0 || intValue > MAX_INTEGER) {
            throw new IllegalArgumentException(
                    "Identifiers can only be constructed from integers between 0 and " + MAX_INTEGER + " (inclusive).");
        }

        final var newValue = new byte[2];

        newValue[0] = (byte) (intValue >> 8);
        newValue[1] = (byte) (intValue);

        return new BeaconIdentifier(newValue);
    }

    /**
     * Creates an identifier from the specified byte array.
     *
     * @param bytes        array to copy from.
     * @param start        the start index, inclusive.
     * @param end          the end index, exclusive.
     * @param littleEndian whether the bytes are ordered in little endian.
     * @return a new identifier.
     * @throws NullPointerException           if bytes is null.
     * @throws ArrayIndexOutOfBoundsException if start or end are outside the bounds of the array.
     * @throws IllegalArgumentException       start is larger than end.
     */
    public static BeaconIdentifier fromBytes(
            final byte[] bytes, final int start, final int end, final boolean littleEndian) {
        if (bytes == null) {
            throw new NullPointerException(
                    "Identifiers cannot be constructed from null pointers but \"bytes\" is null.");
        }
        if (start < 0 || start > bytes.length) {
            throw new ArrayIndexOutOfBoundsException("start < 0 || start > bytes.length");
        }
        if (end > bytes.length) {
            throw new ArrayIndexOutOfBoundsException("end > bytes.length");
        }
        if (start > end) {
            throw new IllegalArgumentException("start > end");
        }

        final var byteRange = Arrays.copyOfRange(bytes, start, end);
        if (littleEndian) {
            reverseArray(byteRange);
        }
        return new BeaconIdentifier(byteRange);
    }

    /**
     * Transforms a {@link UUID} into an identifier.
     * No mangling with strings, only the underlying bytes of the
     * UUID are used so this is fast and stable.
     *
     * @param uuid UUID to create identifier from.
     * @return a new identifier.
     */
    public static BeaconIdentifier fromUuid(final UUID uuid) {
        final var buf = ByteBuffer.allocate(16);
        buf.putLong(uuid.getMostSignificantBits());
        buf.putLong(uuid.getLeastSignificantBits());
        return new BeaconIdentifier(buf.array());
    }

    /**
     * Represents the value as a String. The output varies based on the length of the value.
     * <ul><li>When the value is 2 bytes long: decimal, for example 6536.
     * <li>When the value is 16 bytes long: uuid, for example 2f234454-cf6d-4a0f-adf2-f4911ba9ffa6
     * <li>Else: hexadecimal prefixed with <code>0x</code>, for example 0x0012ab</ul>
     *
     * @return string representation of the current value.
     */
    @Override
    public String toString() {
        // Note:  the toString() method is also used for serialization and deserialization.  So
        // toString() and parse() must always return objects that return true when you call equals()
        if (value == null) {
            return super.toString();
        }

        if (value.length == 2) {
            return Integer.toString(toInt());
        }
        if (value.length == 16) {
            return toUuid().toString();
        }
        return toHexString();
    }

    /**
     * Represents the value as an <code>int</code>.
     *
     * @return value represented as int.
     * @throws UnsupportedOperationException when value length is longer than 2.
     */
    public int toInt() {
        if (value == null) {
            return 0;
        }

        if (value.length > 2) {
            throw new UnsupportedOperationException("Only supported for Identifiers with max byte length of 2");
        }

        var result = 0;
        for (var i = 0; i < value.length; i++) {
            result |= (value[i] & 0xFF) << ((value.length - i - 1) * 8);
        }

        return result;
    }

    /**
     * Converts identifier to a byte array.
     *
     * @param bigEndian true if bytes are MSB first.
     * @return a new byte array with a copy of the value.
     */
    public byte[] toByteArrayOfSpecifiedEndianness(final boolean bigEndian) {
        if (value == null) {
            return null;
        }

        final var copy = Arrays.copyOf(value, value.length);

        if (!bigEndian) {
            reverseArray(copy);
        }

        return copy;
    }

    /**
     * Returns the byte length of this identifier.
     *
     * @return length of identifier.
     */
    public int getByteCount() {
        return value != null ? value.length : 0;
    }

    /**
     * Represents the value as a hexadecimal String. The String is prefixed with <code>0x</code>. For example
     * 0x0034ab.
     *
     * @return value as hexadecimal String.
     */
    public String toHexString() {
        if (value == null) {
            return null;
        }

        final var l = value.length;
        final var out = new char[l * 2 + 2];
        out[0] = '0';
        out[1] = 'x';
        for (int i = 0, j = 2; i < l; i++) {
            out[j] = HEX_DIGITS[(0xF0 & value[i]) >>> 4];
            j++;
            out[j] = HEX_DIGITS[0x0F & value[i]];
            j++;
        }
        return new String(out);
    }

    /**
     * Gives you the identifier as a UUID if possible.
     *
     * @return the identifier as a UUID.
     * @throws UnsupportedOperationException if conversion to UUID fails.
     */
    public UUID toUuid() {
        if (value == null) {
            return null;
        }

        if (value.length != 16) {
            throw new UnsupportedOperationException("Only Identifiers backed by a byte array with length of exactly 16 can be UUIDs.");
        }
        final var buf = ByteBuffer.wrap(value).asLongBuffer();
        return new UUID(buf.get(), buf.get());
    }

    /**
     * Gives you the byte array backing this identifier. Note that identifiers are immutable,
     * so changing that the returned array will not result in a changed identifier.
     *
     * @return a deep copy of the data backing this identifier.
     */
    public byte[] toByteArray() {
        return value != null ? value.clone() : null;
    }

    /**
     * Computes hash code for this instance.
     *
     * @return this instance hash code.
     */
    @Override
    public int hashCode() {
        return value != null ? Arrays.hashCode(value) : 0;
    }

    /**
     * Returns whether both identifiers contain equal value.
     * This is the case when the value is the same and has the same length.
     *
     * @param that object to compare to.
     * @return whether that equals this.
     */
    @Override
    public boolean equals(final Object that) {
        if (!(that instanceof BeaconIdentifier thatIdentifier)) {
            return false;
        }
        return Arrays.equals(value, thatIdentifier.value);
    }

    /**
     * Compares two identifiers.
     * When the identifiers don't have the same length, the identifier having the shortest
     * array is considered smaller than the other.
     *
     * @param that the other identifier.
     * @return 0 if both identifiers are equal. Otherwise, returns -1 or 1 depending on
     * which is bigger than th other.
     * @see Comparable#compareTo(Object)
     */
    @Override
    public int compareTo(final BeaconIdentifier that) {
        if (value.length != that.value.length) {
            return value.length < that.value.length ? -1 : 1;
        }
        for (var i = 0; i < value.length; i++) {
            if (value[i] != that.value[i]) {
                return value[i] < that.value[i] ? -1 : 1;
            }
        }
        return 0;
    }

    /**
     * Reverses provided array.
     *
     * @param bytes array to be reversed.
     */
    private static void reverseArray(final byte[] bytes) {
        for (var i = 0; i < bytes.length / 2; i++) {
            final var mirroredIndex = bytes.length - i - 1;
            final var tmp = bytes[i];
            bytes[i] = bytes[mirroredIndex];
            bytes[mirroredIndex] = tmp;
        }
    }

    /**
     * Parses a string containing a beacon identifier in hexadecimal format.
     *
     * @param identifierString  string to be parsed.
     * @param desiredByteLength length of byte array to create to hold provided value.
     * @return the parsed identifier.
     */
    private static BeaconIdentifier parseHex(final String identifierString, final int desiredByteLength) {
        var str = identifierString.length() % 2 == 0 ? "" : "0";
        str += identifierString.toUpperCase();
        var len = str.length();

        if (desiredByteLength > 0 && desiredByteLength < len / 2) {
            str = str.substring(len - desiredByteLength * 2);
            len = str.length();
        }
        if (desiredByteLength > 0 && desiredByteLength > len / 2) {
            final var extraCharsToAdd = desiredByteLength * 2 - len;
            final var sb = new StringBuilder();
            while (sb.length() < extraCharsToAdd) {
                sb.append("0");
            }
            str = sb + str;
            len = str.length();
        }

        final var result = new byte[len / 2];
        for (var i = 0; i < result.length; i++) {
            result[i] = (byte) (Integer.parseInt(str.substring(i * 2, i * 2 + 2), 16) & 0xFF);
        }
        return new BeaconIdentifier(result);
    }
}
