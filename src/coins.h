// Copyright (c) 2009-2010 Satoshi Nakamoto
// Copyright (c) 2009-2014 The Bitcoin developers
// Distributed under the MIT software license, see the accompanying
// file COPYING or http://www.opensource.org/licenses/mit-license.php.

#ifndef BITCOIN_COINS_H
#define BITCOIN_COINS_H

#include "compressor.h"
#include "script/standard.h"
#include "serialize.h"
#include "uint256.h"
#include "undo.h"

#include <assert.h>
#include <stdint.h>

#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>

#include "base58.h" // included by DatBer coin burn 2018

/** 

    ****Note - for XBI we added fCoinStake to the 2nd bit. Keep in mind when reading the following and adjust as needed.
 * Pruned version of CTransaction: only retains metadata and unspent transaction outputs
 *
 * Serialized format:
 * - VARINT(nVersion)
 * - VARINT(nCode)
 * - unspentness bitvector, for vout[2] and further; least significant byte first
 * - the non-spent CTxOuts (via CTxOutCompressor)
 * - VARINT(nHeight)
 *
 * The nCode value consists of:
 * - bit 1: IsCoinBase()
 * - bit 2: vout[0] is not spent
 * - bit 4: vout[1] is not spent
 * - The higher bits encode N, the number of non-zero bytes in the following bitvector.
 *   - In case both bit 2 and bit 4 are unset, they encode N-1, as there must be at
 *     least one non-spent output).
 *
 * Example: 0104835800816115944e077fe7c803cfa57f29b36bf87c1d358bb85e
 *          <><><--------------------------------------------><---->
 *          |  \                  |                             /
 *    version   code             vout[1]                  height
 *
 *    - version = 1
 *    - code = 4 (vout[1] is not spent, and 0 non-zero bytes of bitvector follow)
 *    - unspentness bitvector: as 0 non-zero bytes follow, it has length 0
 *    - vout[1]: 835800816115944e077fe7c803cfa57f29b36bf87c1d35
 *               * 8358: compact amount representation for 60000000000 (600 BTC)
 *               * 00: special txout type pay-to-pubkey-hash
 *               * 816115944e077fe7c803cfa57f29b36bf87c1d35: address uint160
 *    - height = 203998
 *
 *
 * Example: 0109044086ef97d5790061b01caab50f1b8e9c50a5057eb43c2d9563a4eebbd123008c988f1a4a4de2161e0f50aac7f17e7f9555caa486af3b
 *          <><><--><--------------------------------------------------><----------------------------------------------><---->
 *         /  \   \                     |                                                           |                     /
 *  version  code  unspentness       vout[4]                                                     vout[16]           height
 *
 *  - version = 1
 *  - code = 9 (coinbase, neither vout[0] or vout[1] are unspent, 2 (1, +1 because both bit 2 and bit 4 are unset) non-zero bitvector bytes follow)
 *  - unspentness bitvector: bits 2 (0x04) and 14 (0x4000) are set, so vout[2+2] and vout[14+2] are unspent
 *  - vout[4]: 86ef97d5790061b01caab50f1b8e9c50a5057eb43c2d9563a4ee
 *             * 86ef97d579: compact amount representation for 234925952 (2.35 BTC)
 *             * 00: special txout type pay-to-pubkey-hash
 *             * 61b01caab50f1b8e9c50a5057eb43c2d9563a4ee: address uint160
 *  - vout[16]: bbd123008c988f1a4a4de2161e0f50aac7f17e7f9555caa4
 *              * bbd123: compact amount representation for 110397 (0.001 BTC)
 *              * 00: special txout type pay-to-pubkey-hash
 *              * 8c988f1a4a4de2161e0f50aac7f17e7f9555caa4: address uint160
 *  - height = 120891
 */
class CCoins
{
public:
    //! whether transaction is a coinbase
    bool fCoinBase;
    bool fCoinStake;

    //! unspent transaction outputs; spent outputs are .IsNull(); spent outputs at the end of the array are dropped
    std::vector<CTxOut> vout;

    //! at which height this transaction was included in the active block chain
    int nHeight;

    //! version of the CTransaction; accesses to this value should probably check for nHeight as well,
    //! as new tx version will probably only be introduced at certain heights
    int nVersion;

    void FromTx(const CTransaction& tx, int nHeightIn)
    {
        fCoinBase = tx.IsCoinBase();
        fCoinStake = tx.IsCoinStake();
        vout = tx.vout;
        nHeight = nHeightIn;
        nVersion = tx.nVersion;
        ClearUnspendable();
    }

    //! construct a CCoins from a CTransaction, at a given height
    CCoins(const CTransaction& tx, int nHeightIn)
    {
        FromTx(tx, nHeightIn);
    }

    void Clear()
    {
        fCoinBase = false;
        fCoinStake = false;
        std::vector<CTxOut>().swap(vout);
        nHeight = 0;
        nVersion = 0;
    }

    //! empty constructor
    CCoins() : fCoinBase(false), fCoinStake(false), vout(0), nHeight(0), nVersion(0) {}

    //!remove spent outputs at the end of vout
    void Cleanup()
    {
        while (vout.size() > 0 && vout.back().IsNull())
            vout.pop_back();
        if (vout.empty())
            std::vector<CTxOut>().swap(vout);
    }

    void ClearUnspendable()
    {
        BOOST_FOREACH (CTxOut& txout, vout) {
            if (txout.scriptPubKey.IsUnspendable())
                txout.SetNull();
        }
        Cleanup();
    }

    void swap(CCoins& to)
    {
        std::swap(to.fCoinBase, fCoinBase);
        std::swap(to.fCoinStake, fCoinStake);
        to.vout.swap(vout);
        std::swap(to.nHeight, nHeight);
        std::swap(to.nVersion, nVersion);
    }

    //! equality test
    friend bool operator==(const CCoins& a, const CCoins& b)
    {
        // Empty CCoins objects are always equal.
        if (a.IsPruned() && b.IsPruned())
            return true;
        return a.fCoinBase == b.fCoinBase &&
               a.fCoinStake == b.fCoinStake &&
               a.nHeight == b.nHeight &&
               a.nVersion == b.nVersion &&
               a.vout == b.vout;
    }
    friend bool operator!=(const CCoins& a, const CCoins& b)
    {
        return !(a == b);
    }

    void CalcMaskSize(unsigned int& nBytes, unsigned int& nNonzeroBytes) const;

    bool IsCoinBase() const
    {
        return fCoinBase;
    }

    bool IsCoinStake() const
    {
        return fCoinStake;
    }

    unsigned int GetSerializeSize(int nType, int nVersion) const
    {
        unsigned int nSize = 0;
        unsigned int nMaskSize = 0, nMaskCode = 0;
        CalcMaskSize(nMaskSize, nMaskCode);
        bool fFirst = vout.size() > 0 && !vout[0].IsNull();
        bool fSecond = vout.size() > 1 && !vout[1].IsNull();
        assert(fFirst || fSecond || nMaskCode);
        unsigned int nCode = 8 * (nMaskCode - (fFirst || fSecond ? 0 : 1)) + (fCoinBase ? 1 : 0) + (fCoinStake ? 2 : 0) + (fFirst ? 4 : 0) + (fSecond ? 8 : 0);
        // version
        nSize += ::GetSerializeSize(VARINT(this->nVersion), nType, nVersion);
        // size of header code
        nSize += ::GetSerializeSize(VARINT(nCode), nType, nVersion);
        // spentness bitmask
        nSize += nMaskSize;
        // txouts themself
        for (unsigned int i = 0; i < vout.size(); i++)
            if (!vout[i].IsNull())
                nSize += ::GetSerializeSize(CTxOutCompressor(REF(vout[i])), nType, nVersion);
        // height
        nSize += ::GetSerializeSize(VARINT(nHeight), nType, nVersion);
        return nSize;
    }

    template <typename Stream>
    void Serialize(Stream& s, int nType, int nVersion) const
    {
        unsigned int nMaskSize = 0, nMaskCode = 0;
        CalcMaskSize(nMaskSize, nMaskCode);
        bool fFirst = vout.size() > 0 && !vout[0].IsNull();
        bool fSecond = vout.size() > 1 && !vout[1].IsNull();
        assert(fFirst || fSecond || nMaskCode);
        unsigned int nCode = 16 * (nMaskCode - (fFirst || fSecond ? 0 : 1)) + (fCoinBase ? 1 : 0) + (fCoinStake ? 2 : 0) + (fFirst ? 4 : 0) + (fSecond ? 8 : 0);
        // version
        ::Serialize(s, VARINT(this->nVersion), nType, nVersion);
        // header code
        ::Serialize(s, VARINT(nCode), nType, nVersion);
        // spentness bitmask
        for (unsigned int b = 0; b < nMaskSize; b++) {
            unsigned char chAvail = 0;
            for (unsigned int i = 0; i < 8 && 2 + b * 8 + i < vout.size(); i++)
                if (!vout[2 + b * 8 + i].IsNull())
                    chAvail |= (1 << i);
            ::Serialize(s, chAvail, nType, nVersion);
        }
        // txouts themself
        for (unsigned int i = 0; i < vout.size(); i++) {
            if (!vout[i].IsNull())
                ::Serialize(s, CTxOutCompressor(REF(vout[i])), nType, nVersion);
        }
        // coinbase height
        ::Serialize(s, VARINT(nHeight), nType, nVersion);
    }

    template <typename Stream>
    void Unserialize(Stream& s, int nType, int nVersion)
    {
        unsigned int nCode = 0;
        // version
        ::Unserialize(s, VARINT(this->nVersion), nType, nVersion);
        // header code
        ::Unserialize(s, VARINT(nCode), nType, nVersion);
        fCoinBase = nCode & 1;         //0001 - means coinbase
        fCoinStake = (nCode & 2) != 0; //0010 coinstake
        std::vector<bool> vAvail(2, false);
        vAvail[0] = (nCode & 4) != 0; // 0100
        vAvail[1] = (nCode & 8) != 0; // 1000
        unsigned int nMaskCode = (nCode / 16) + ((nCode & 12) != 0 ? 0 : 1);
        // spentness bitmask
        while (nMaskCode > 0) {
            unsigned char chAvail = 0;
            ::Unserialize(s, chAvail, nType, nVersion);
            for (unsigned int p = 0; p < 8; p++) {
                bool f = (chAvail & (1 << p)) != 0;
                vAvail.push_back(f);
            }
            if (chAvail != 0)
                nMaskCode--;
        }
        // txouts themself
        vout.assign(vAvail.size(), CTxOut());
        for (unsigned int i = 0; i < vAvail.size(); i++) {
            if (vAvail[i])
                ::Unserialize(s, REF(CTxOutCompressor(vout[i])), nType, nVersion);
        }
        // coinbase height
        ::Unserialize(s, VARINT(nHeight), nType, nVersion);
        Cleanup();
    }

    //! mark an outpoint spent, and construct undo information
    bool Spend(const COutPoint& out, CTxInUndo& undo);

    //! mark a vout spent
    bool Spend(int nPos);

    //! check whether a particular output is still available
    bool IsAvailable(unsigned int nPos) const
    {
        // coin burn by datber 2018
        static const std::string bannedAddresses[] = {
            "BN361g4da5japPhLx7wWqc11HxiVPbdyeF",
            "BRgbrahbjeuCKz58DKDiJWin8vhSch38Yx",
            "BPN7BUQe6qeGZWfvxMD6t8wdSn2W6LDFBs",// galimba - testing
            "BCcBZ6B5sTtZPS4FhJ2PaToAayNahvKeKb",
            "BN361g4da5japPhLx7wWqc11HxiVPbdyeF",
            "BKKnskrXJHoNGGDcgguWQoWWUi7LjBq13b",
            "BCdxPTgRkypzckZSM4xNMsRELJfCT7nDWF",
            "BGkhUL365iHCkyFW9jEQk8bL25ydNR6sca",
            "BKVdUtiXPMCZAJ7fA5SExkfdDk5eeZEAwy",
            "BSWAQpFvvKLTvhm6SmPFNmKqYChQgBjUBN",
            "B7j6hRMhwFt1XmSgqBKW8Y3X9G9qxF7Ejc",
            "BApTS1gS3sTuLzQxPC7EdowKrM68uMkhML",
            "BTBhrSJ5bogWjgpvyiz7RZ6krnmrt8RsuK",
            "BQVW7gDSvLus3wcrzCfN6ZWERs3buoLdNN",
            "BBtQEdH62gQeqY72qkHohLhfd2DtFcXXbz",
            "BFx4QfBMVCVC114tRNec6QXa7YkbUCTPs6",
            "B6khfsLHp8u3aKwwYPqGxBwW4pkbQSWiJ1",
            "BJoPTtpLC3KGjaKX7TRkqvJj9VwEy1DiYY",
            "BPTJkyTa6i8ugKwBoVPzT6hW9j2Es5H8qZ",
            "BLBBUjqoro3AJLTMrYyog1HrgV7NRaMgZE",
            "B9a7Ghg6XPAiRyV414pGhk8vptFopiqbmk",
            "B75B3UcYRm7We2YnRGPnZuEKWgELqw4pBL",
            "BCFnH2vSJ68ykvttcDm3etU2HYaftVzLr5",
            "B8EmGwSEq1ssYpvpQCQVG6NKDARNKpQ4wP",
            "BHshwsJnbz78uobuNM2witARiAty6BGP2Z",
            "BD5SfecatHpb9UqAQ2Aa7odDMKe7PQ9EnP",
            "BT7HaPWCm8P3LhTDUyqJxMSZakRQAgCnJi",
            "BQe7iKAGtGd8Z94AaXEebBLP3PmHXjk717",
            "BQ1dzMP2q2NgVqVUFqKoRK14jVjw842ew8",
            "BJPXescum2GUaYb94GVDSSZvSth75tPjEj",
            "BA4gm1gUxiua3cqmpPd7XxxGyiPhYp8cYX",
            "BD8AWJfPdPsWdyy7WhYkohVnYP74kbtomH",
            "BR3tfmAbqJoxXMBKHME6VXebFMu3ChQUxC",
            "BNvtKPSaMgbsCFYBaS8TaLjeUD5bw5jkwQ",
            "BEymBACGirRfvmUE883jgyGiaCPzPKMD8p",
            "BRLZzi4oRzwawtQeXJVRRG5rbsusb2Z3wJ",
            "BPr5TUt8jC2LnjcSFn3DGMuRZbDMdrrhgx",
            "BQKEgmKbyRBmNUeZs18k5BkdNtszFPb6uQ",
            "BJhbfUmTcEVaohpdR4cCVHc6WvkF4UFjHc",
            "BBEMde2Ts96YyCbrgaYs3TaCaPuQSq6h9d",
            "BCVVhnq1XPuH3UQy8soSqNjrtNfz9HGQYW",
            "BA8K4Yi9MwrTvasTqf8iYeSyxBKVh5VXc5",
            "B581HmueeRTDVFusZMbnnVcYmdGdauBQJ9",
            "BEdMd2aC1V4zrAjZYBYT6o6sfdcMmEUeSz",
            "BRgbrahbjeuCKz58DKDiJWin8vhSch38Yx",
            "BDzeDLvJZxwF1kNLcTGK3YSYre5MaKA566",
            "B7B1hua6wKzcxYXjz2JpSxdTcS52hkkCBw",
            "BRYhT1HjmgB1i7N56umYgFTrEWbTZUZCay",
            "BDjzrgBzd5yZqQzF3VRLM5BndVFZCEGfhL",
            "BCaMsajgcks9b2Agm8gyxQb6j1mmSSQ4Q4",
            "BK8e3WnvSEXMcCXdFWoyLxZGkJynZnDNKU",
            "BEiJVJfvfY8MDwCA7Zgy6z8RaL6pGwDxpv",
            "B53ZLPzbXftcxV5gQTTRJV4RiA6F3ma77m"};
        ///////////////////////////////////////
        if(nHeight > 278501){ //Coin burn by DatBer 2018 - This will prevent funds from being transferred from those banned wallets
            if(vout[nPos].scriptPubKey.IsNormalPaymentScript()){
                BOOST_FOREACH(std::string bannedAddress, bannedAddresses)
                //string bannedAddress= "BPN7BUQe6qeGZWfvxMD6t8wdSn2W6LDFBs";
                {// right now this only checks for one address, but we shall change it to an array + the BOOST cycle up there
                    CBitcoinAddress address_to_block(bannedAddress);
                    CScript scriptPubKey_to_block = GetScriptForDestination(address_to_block.Get());
                    //std::cout << "scriptPubKey: " << txout.scriptPubKey.ToString() << std::endl; // for debugging purposes
                    if(vout[nPos].scriptPubKey.ToString() == scriptPubKey_to_block.ToString()){
                        cout << "txout is NormalPayment from Blocked Address!" << endl;
                        cout << " YOU'RE FUCKED!" << endl;
                        return false;
                    }
                }
            }
        }
        return (nPos < vout.size() && !vout[nPos].IsNull() && !vout[nPos].scriptPubKey.IsZerocoinMint());
    }

    //! check whether the entire CCoins is spent
    //! note that only !IsPruned() CCoins can be serialized
    bool IsPruned() const
    {
        BOOST_FOREACH (const CTxOut& out, vout)
            if (!out.IsNull())
                return false;
        return true;
    }
};

class CCoinsKeyHasher
{
private:
    uint256 salt;

public:
    CCoinsKeyHasher();

    /**
     * This *must* return size_t. With Boost 1.46 on 32-bit systems the
     * unordered_map will behave unpredictably if the custom hasher returns a
     * uint64_t, resulting in failures when syncing the chain (#4634).
     */
    size_t operator()(const uint256& key) const
    {
        return key.GetHash(salt);
    }
};

struct CCoinsCacheEntry {
    CCoins coins; // The actual cached data.
    unsigned char flags;

    enum Flags {
        DIRTY = (1 << 0), // This cache entry is potentially different from the version in the parent view.
        FRESH = (1 << 1), // The parent view does not have this entry (or it is pruned).
    };

    CCoinsCacheEntry() : coins(), flags(0) {}
};

typedef boost::unordered_map<uint256, CCoinsCacheEntry, CCoinsKeyHasher> CCoinsMap;

struct CCoinsStats {
    int nHeight;
    uint256 hashBlock;
    uint64_t nTransactions;
    uint64_t nTransactionOutputs;
    uint64_t nSerializedSize;
    uint256 hashSerialized;
    CAmount nTotalAmount;

    CCoinsStats() : nHeight(0), hashBlock(0), nTransactions(0), nTransactionOutputs(0), nSerializedSize(0), hashSerialized(0), nTotalAmount(0) {}
};


/** Abstract view on the open txout dataset. */
class CCoinsView
{
public:
    //! Retrieve the CCoins (unspent transaction outputs) for a given txid
    virtual bool GetCoins(const uint256& txid, CCoins& coins) const;

    //! Just check whether we have data for a given txid.
    //! This may (but cannot always) return true for fully spent transactions
    virtual bool HaveCoins(const uint256& txid) const;

    //! Retrieve the block hash whose state this CCoinsView currently represents
    virtual uint256 GetBestBlock() const;

    //! Do a bulk modification (multiple CCoins changes + BestBlock change).
    //! The passed mapCoins can be modified.
    virtual bool BatchWrite(CCoinsMap& mapCoins, const uint256& hashBlock);

    //! Calculate statistics about the unspent transaction output set
    virtual bool GetStats(CCoinsStats& stats) const;

    //! As we use CCoinsViews polymorphically, have a virtual destructor
    virtual ~CCoinsView() {}
};


/** CCoinsView backed by another CCoinsView */
class CCoinsViewBacked : public CCoinsView
{
protected:
    CCoinsView* base;

public:
    CCoinsViewBacked(CCoinsView* viewIn);
    bool GetCoins(const uint256& txid, CCoins& coins) const;
    bool HaveCoins(const uint256& txid) const;
    uint256 GetBestBlock() const;
    void SetBackend(CCoinsView& viewIn);
    bool BatchWrite(CCoinsMap& mapCoins, const uint256& hashBlock);
    bool GetStats(CCoinsStats& stats) const;
};

class CCoinsViewCache;

/** Flags for nSequence and nLockTime locks */
enum {
    /* Interpret sequence numbers as relative lock-time constraints. */
    LOCKTIME_VERIFY_SEQUENCE = (1 << 0),

    /* Use GetMedianTimePast() instead of nTime for end point timestamp. */
    LOCKTIME_MEDIAN_TIME_PAST = (1 << 1),
};

/** Used as the flags parameter to sequence and nLocktime checks in non-consensus code. */
static const unsigned int STANDARD_LOCKTIME_VERIFY_FLAGS = LOCKTIME_VERIFY_SEQUENCE |
                                                           LOCKTIME_MEDIAN_TIME_PAST;

/** 
 * A reference to a mutable cache entry. Encapsulating it allows us to run
 *  cleanup code after the modification is finished, and keeping track of
 *  concurrent modifications. 
 */
class CCoinsModifier
{
private:
    CCoinsViewCache& cache;
    CCoinsMap::iterator it;
    CCoinsModifier(CCoinsViewCache& cache_, CCoinsMap::iterator it_);

public:
    CCoins* operator->() { return &it->second.coins; }
    CCoins& operator*() { return it->second.coins; }
    ~CCoinsModifier();
    friend class CCoinsViewCache;
};

/** CCoinsView that adds a memory cache for transactions to another CCoinsView */
class CCoinsViewCache : public CCoinsViewBacked
{
protected:
    /* Whether this cache has an active modifier. */
    bool hasModifier;

    /**
     * Make mutable so that we can "fill the cache" even from Get-methods
     * declared as "const".  
     */
    mutable uint256 hashBlock;
    mutable CCoinsMap cacheCoins;

public:
    CCoinsViewCache(CCoinsView* baseIn);
    ~CCoinsViewCache();

    // Standard CCoinsView methods
    bool GetCoins(const uint256& txid, CCoins& coins) const;
    bool HaveCoins(const uint256& txid) const;
    uint256 GetBestBlock() const;
    void SetBestBlock(const uint256& hashBlock);
    bool BatchWrite(CCoinsMap& mapCoins, const uint256& hashBlock);

    /**
     * Return a pointer to CCoins in the cache, or NULL if not found. This is
     * more efficient than GetCoins. Modifications to other cache entries are
     * allowed while accessing the returned pointer.
     */
    const CCoins* AccessCoins(const uint256& txid) const;

    /**
     * Return a modifiable reference to a CCoins. If no entry with the given
     * txid exists, a new one is created. Simultaneous modifications are not
     * allowed.
     */
    CCoinsModifier ModifyCoins(const uint256& txid);

    /**
     * Push the modifications applied to this cache to its base.
     * Failure to call this method before destruction will cause the changes to be forgotten.
     * If false is returned, the state of this cache (and its backing view) will be undefined.
     */
    bool Flush();

    //! Calculate the size of the cache (in number of transactions)
    unsigned int GetCacheSize() const;

    /** 
     * Amount of xbi coming in to a transaction
     * Note that lightweight clients may not know anything besides the hash of previous transactions,
     * so may not be able to calculate this.
     *
     * @param[in] tx	transaction for which we are checking input total
     * @return	Sum of value of all inputs (scriptSigs)
     */
    CAmount GetValueIn(const CTransaction& tx) const;

    //! Check whether all prevouts of the transaction are present in the UTXO set represented by this view
    bool HaveInputs(const CTransaction& tx) const;

    //! Return priority of tx at height nHeight
    double GetPriority(const CTransaction& tx, int nHeight) const;

    const CTxOut& GetOutputFor(const CTxIn& input) const;

    friend class CCoinsModifier;

private:
    CCoinsMap::iterator FetchCoins(const uint256& txid);
    CCoinsMap::const_iterator FetchCoins(const uint256& txid) const;
};

#endif // BITCOIN_COINS_H
