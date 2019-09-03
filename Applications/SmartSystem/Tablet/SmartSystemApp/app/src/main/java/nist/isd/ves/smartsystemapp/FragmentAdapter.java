package nist.isd.ves.smartsystemapp;

import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;

/**
 * Created by ves on 7/16/2015.
 * puts fragments in layout for MainActivity essentially
 */
public class FragmentAdapter extends FragmentPagerAdapter {


    public FragmentAdapter(FragmentManager fm) {
        super(fm);
    }
    @Override
    public CharSequence getPageTitle(int position) {
        return (position == 0)? "" : "" ;
    }

    @Override
    public int getCount() {
        return 2;
    }
    @Override
    public Fragment getItem(int position) {
        return (position == 0)? new ProcessFragment() : new VideoFragment() ;
    }
}
